//! Local Axum repair service.
//!
//! Geometry crates stay free of Axum. This crate is the only HTTP adapter.
//! It binds to localhost, caps upload size and concurrency, repairs on a
//! blocking thread, and drops the validation tempfile after the roundtrip check.

use axum::body::Bytes;
use axum::extract::{DefaultBodyLimit, Query, State};
use axum::http::{HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use mesh_analysis::analyze;
use mesh_core::{MeshError, RepairProfile, ResourceLimits};
use mesh_formats::{load_model, save_model};
use mesh_repair::repair_mesh;
use mesh_validation::{RoundtripRequest, verify_roundtrip};
use serde::Deserialize;
use serde::Serialize;
use std::path::Path;
use std::sync::Arc;
use tokio::sync::Semaphore;

/// Quotas for the local repair service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServiceLimits {
    pub max_upload_bytes: u64,
    pub default_profile: RepairProfile,
    pub max_concurrent: usize,
}

impl Default for ServiceLimits {
    fn default() -> Self {
        Self {
            max_upload_bytes: 64 * 1024 * 1024,
            default_profile: RepairProfile::Conservative,
            max_concurrent: 2,
        }
    }
}

#[derive(Clone)]
struct AppState {
    limits: ServiceLimits,
    slots: Arc<Semaphore>,
}

#[derive(Debug, Deserialize)]
struct RepairQuery {
    profile: Option<String>,
}

#[derive(Debug, Serialize)]
struct RepairBody {
    profile: String,
    extension: String,
    was_noop: bool,
    watertight: bool,
    validation_passed: bool,
    model_base64: String,
    warnings: Vec<String>,
    notes: Vec<String>,
}

#[derive(Debug, Serialize)]
struct ErrorBody {
    error: String,
}

/// Router for `GET /health` and `POST /api/repair`.
pub fn router(limits: ServiceLimits) -> Router {
    let state = AppState {
        slots: Arc::new(Semaphore::new(limits.max_concurrent.max(1))),
        limits,
    };
    let limit = usize::try_from(limits.max_upload_bytes).unwrap_or(usize::MAX);
    Router::new()
        .route("/health", get(health))
        .route("/api/repair", post(repair))
        .layer(DefaultBodyLimit::max(limit))
        .with_state(state)
}

async fn health() -> Json<serde_json::Value> {
    Json(serde_json::json!({ "status": "ok" }))
}

async fn repair(
    State(state): State<AppState>,
    Query(query): Query<RepairQuery>,
    headers: HeaderMap,
    body: Bytes,
) -> Response {
    let Ok(permit) = state.slots.clone().try_acquire_owned() else {
        return error(StatusCode::TOO_MANY_REQUESTS, "repair service is busy");
    };
    let filename = headers
        .get("x-filename")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
        .to_string();
    let profile = match resolve_profile(query.profile.as_deref(), state.limits.default_profile) {
        Ok(profile) => profile,
        Err(message) => return error(StatusCode::BAD_REQUEST, &message),
    };
    let limits = state.limits;
    let bytes = body.to_vec();
    let joined = tokio::task::spawn_blocking(move || {
        let _permit = permit;
        repair_upload(&filename, &bytes, profile, limits)
    })
    .await;
    match joined {
        Ok(Ok(body)) => (StatusCode::OK, Json(body)).into_response(),
        Ok(Err(failure)) => error(failure.status, &failure.message),
        Err(_) => error(StatusCode::INTERNAL_SERVER_ERROR, "repair task failed"),
    }
}

struct Failure {
    status: StatusCode,
    message: String,
}

fn repair_upload(
    filename: &str,
    bytes: &[u8],
    profile: RepairProfile,
    limits: ServiceLimits,
) -> Result<RepairBody, Failure> {
    let ext = Path::new(filename)
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase)
        .unwrap_or_default();
    if !matches!(ext.as_str(), "stl" | "obj" | "ply" | "amf" | "gltf" | "glb") {
        return Err(fail(
            StatusCode::UNSUPPORTED_MEDIA_TYPE,
            "X-Filename must end in .stl, .obj, .ply, .amf, .gltf, or .glb. 3MF, SVG, and STEP stay on the CLI and desktop app.",
        ));
    }
    if bytes.len() as u64 > limits.max_upload_bytes {
        return Err(fail(
            StatusCode::PAYLOAD_TOO_LARGE,
            "upload exceeds the service limit",
        ));
    }
    let dir = tempfile::tempdir()
        .map_err(|err| fail(StatusCode::INTERNAL_SERVER_ERROR, &err.to_string()))?;
    let input = dir.path().join(format!("input.{ext}"));
    std::fs::write(&input, bytes)
        .map_err(|err| fail(StatusCode::INTERNAL_SERVER_ERROR, &err.to_string()))?;
    let resource = ResourceLimits {
        max_file_bytes: limits.max_upload_bytes,
        ..ResourceLimits::default()
    };
    let loaded = load_model(&input, &resource).map_err(mesh_failure)?;
    let repaired =
        repair_mesh(&loaded.mesh, &loaded.attributes, profile, None).map_err(mesh_failure)?;
    let expected = analyze(&repaired.mesh);
    let output = dir.path().join(format!("repaired.{ext}"));
    save_model(&output, &repaired.mesh, &repaired.attributes).map_err(mesh_failure)?;
    let validation = verify_roundtrip(RoundtripRequest {
        ext: &ext,
        original_mesh: &loaded.mesh,
        repaired_mesh: &repaired.mesh,
        repaired_attrs: &repaired.attributes,
        expected_report: &expected,
        profile,
        was_noop: repaired.report.was_noop,
        limits: &resource,
    })
    .map_err(mesh_failure)?;
    if !validation.passed {
        return Err(fail(
            StatusCode::UNPROCESSABLE_ENTITY,
            &validation.errors.join("; "),
        ));
    }
    let model = std::fs::read(&output)
        .map_err(|err| fail(StatusCode::INTERNAL_SERVER_ERROR, &err.to_string()))?;
    drop(dir);
    Ok(RepairBody {
        profile: profile_name(profile).to_string(),
        extension: ext,
        was_noop: repaired.report.was_noop,
        watertight: repaired.report.after.is_watertight,
        validation_passed: true,
        model_base64: b64_encode(&model),
        warnings: repaired.report.warnings,
        notes: repaired.report.unresolved_notes,
    })
}

fn resolve_profile(raw: Option<&str>, default: RepairProfile) -> Result<RepairProfile, String> {
    match raw.map(str::to_ascii_lowercase).as_deref() {
        None => Ok(default),
        Some("conservative") => Ok(RepairProfile::Conservative),
        Some("standard") => Ok(RepairProfile::Standard),
        Some("aggressive") => Ok(RepairProfile::Aggressive),
        Some(other) => Err(format!("unknown repair profile '{other}'")),
    }
}

fn profile_name(profile: RepairProfile) -> &'static str {
    match profile {
        RepairProfile::Conservative => "conservative",
        RepairProfile::Standard => "standard",
        RepairProfile::Aggressive => "aggressive",
    }
}

fn mesh_failure(err: MeshError) -> Failure {
    fail(StatusCode::BAD_REQUEST, &err.to_string())
}

fn fail(status: StatusCode, message: &str) -> Failure {
    Failure {
        status,
        message: message.to_string(),
    }
}

fn error(status: StatusCode, message: &str) -> Response {
    (
        status,
        Json(ErrorBody {
            error: message.to_string(),
        }),
    )
        .into_response()
}

fn b64_encode(bytes: &[u8]) -> String {
    const T: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    let mut i = 0;
    while i + 3 <= bytes.len() {
        let n = ((bytes[i] as u32) << 16) | ((bytes[i + 1] as u32) << 8) | bytes[i + 2] as u32;
        out.push(T[((n >> 18) & 63) as usize] as char);
        out.push(T[((n >> 12) & 63) as usize] as char);
        out.push(T[((n >> 6) & 63) as usize] as char);
        out.push(T[(n & 63) as usize] as char);
        i += 3;
    }
    if i < bytes.len() {
        let rem = bytes.len() - i;
        let mut n = (bytes[i] as u32) << 16;
        if rem == 2 {
            n |= (bytes[i + 1] as u32) << 8;
        }
        out.push(T[((n >> 18) & 63) as usize] as char);
        out.push(T[((n >> 12) & 63) as usize] as char);
        if rem == 2 {
            out.push(T[((n >> 6) & 63) as usize] as char);
            out.push('=');
        } else {
            out.push('=');
            out.push('=');
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use mesh_core::TriangleMesh;
    use mesh_formats::write_stl_binary_path;
    use nalgebra::Point3;
    use tower::ServiceExt;

    #[test]
    fn default_limits_use_conservative_profile() {
        let limits = ServiceLimits::default();
        assert_eq!(limits.default_profile, RepairProfile::Conservative);
        assert_eq!(limits.max_concurrent, 2);
    }

    fn triangle_stl() -> Vec<u8> {
        let mesh = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let dir = tempfile::tempdir().expect("temp");
        let path = dir.path().join("tri.stl");
        write_stl_binary_path(&path, &mesh).expect("stl");
        std::fs::read(path).expect("read")
    }

    #[tokio::test]
    async fn health_and_repair_roundtrip() {
        let app = router(ServiceLimits::default());
        let health = app
            .clone()
            .oneshot(
                Request::builder()
                    .uri("/health")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(health.status(), StatusCode::OK);

        let response = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/repair?profile=conservative")
                    .header("x-filename", "tri.stl")
                    .header("content-type", "application/octet-stream")
                    .body(Body::from(triangle_stl()))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let bytes = axum::body::to_bytes(response.into_body(), 8 * 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(json["validation_passed"], true);
        assert!(json["model_base64"].as_str().unwrap().len() > 8);
    }

    #[tokio::test]
    async fn busy_slot_returns_429() {
        let limits = ServiceLimits {
            max_concurrent: 1,
            ..ServiceLimits::default()
        };
        let app = router(limits);
        let _hold = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/repair")
                    .header("x-filename", "tri.stl")
                    .body(Body::from(triangle_stl()))
                    .unwrap(),
            )
            .await
            .unwrap();
        // The first request has finished, so the slot is free. A second call with
        // an exhausted semaphore is covered by try_acquire; force it by filling
        // the semaphore before the request starts.
        let state_full = AppState {
            limits,
            slots: Arc::new(Semaphore::new(0)),
        };
        let blocked = Router::new()
            .route("/api/repair", post(repair))
            .with_state(state_full)
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/repair")
                    .header("x-filename", "tri.stl")
                    .body(Body::from(triangle_stl()))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(blocked.status(), StatusCode::TOO_MANY_REQUESTS);
        let _ = _hold;
    }
}
