use clap::{Parser, Subcommand, ValueEnum};
use mesh_analysis::{DiagnosticReport, analyze};
use mesh_core::{RepairProfile, ResourceLimits};
use mesh_formats::{RepairedThreeMfPart, ThreeMfKind, ThreeMfPackage, load_model, save_model};
use mesh_repair::{RepairResult, repair_mesh};
use mesh_validation::{RoundtripRequest, verify_roundtrip, verify_threemf_roundtrip};
use std::path::{Path, PathBuf};
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(
    name = "mesh-repair",
    version,
    about = "Diagnose and repair 3D printing meshes while preserving 3MF painting and face correspondence"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum ProfileArg {
    Conservative,
    Standard,
    Aggressive,
}

impl From<ProfileArg> for RepairProfile {
    fn from(value: ProfileArg) -> Self {
        match value {
            ProfileArg::Conservative => Self::Conservative,
            ProfileArg::Standard => Self::Standard,
            ProfileArg::Aggressive => Self::Aggressive,
        }
    }
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Inspect a mesh file (.3mf, .stl, .obj) without modifying anything on disk.
    Analyze {
        input: PathBuf,
        #[arg(long)]
        json: bool,
    },
    /// Repair a mesh and write a validated copy to a new file.
    Repair {
        input: PathBuf,
        #[arg(long, value_enum, default_value_t = ProfileArg::Conservative)]
        profile: ProfileArg,
        #[arg(short, long)]
        output: Option<PathBuf>,
        #[arg(long)]
        json: bool,
    },
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .with_writer(std::io::stderr)
        .init();

    let cli = Cli::parse();
    if let Err(err) = run(cli) {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn std::error::Error>> {
    let limits = ResourceLimits::default();
    match cli.command {
        Command::Analyze { input, json } => {
            let loaded = load_model(&input, &limits)?;
            if let Some(pkg) = &loaded.threemf {
                let part_reports: Vec<DiagnosticReport> =
                    pkg.parts.iter().map(|p| analyze(&p.mesh)).collect();
                if json {
                    let payload = serde_json::json!({
                        "file": input,
                        "kind": format!("{:?}", pkg.kind),
                        "unit": pkg.unit,
                        "filament_slots": pkg.filament_palette.len(),
                        "parts": pkg.parts.iter().zip(part_reports.iter()).map(|(p, r)| {
                            serde_json::json!({
                                "archive_path": p.archive_path,
                                "object_id": p.object_id,
                                "object_name": p.object_name,
                                "painted_faces": p.attributes.faces.iter().filter(|f| f.paint.is_some()).count(),
                                "report": r,
                            })
                        }).collect::<Vec<_>>(),
                        "warnings": loaded.warnings,
                    });
                    println!("{}", serde_json::to_string_pretty(&payload)?);
                } else {
                    println!(
                        "File: {} (3MF {:?}, unit: {})",
                        input.display(),
                        pkg.kind,
                        pkg.unit
                    );
                    println!(
                        "Objects: {} | Filament palette colors: {}",
                        pkg.parts.len(),
                        pkg.filament_palette.len()
                    );
                    for (p, r) in pkg.parts.iter().zip(part_reports.iter()) {
                        let painted = p
                            .attributes
                            .faces
                            .iter()
                            .filter(|f| f.paint.is_some())
                            .count();
                        println!(
                            "\n-- Object {} ({}) [{} | {} painted face(s)] --",
                            p.object_id,
                            p.object_name.as_deref().unwrap_or("unnamed"),
                            p.archive_path,
                            painted
                        );
                        println!("{}", r.summary_text());
                    }
                    for w in &loaded.warnings {
                        println!("  [WARN] {w}");
                    }
                }
            } else {
                let report = analyze(&loaded.mesh);
                if json {
                    println!("{}", serde_json::to_string_pretty(&report)?);
                } else {
                    println!("File: {}", input.display());
                    println!("{}", report.summary_text());
                    for w in &loaded.warnings {
                        println!("  [WARN] {w}");
                    }
                }
            }
        }
        Command::Repair {
            input,
            profile,
            output,
            json,
        } => {
            let profile: RepairProfile = profile.into();
            let out_path = output.unwrap_or_else(|| default_repaired_path(&input));
            if paths_match(&input, &out_path) {
                return Err(
                    "refusing to overwrite the original input file; specify a different -o path"
                        .into(),
                );
            }

            let loaded = load_model(&input, &limits)?;
            if let Some(pkg) = &loaded.threemf {
                repair_threemf_cli(pkg, &input, &out_path, profile, json, &limits)?;
                return Ok(());
            }

            let result = repair_mesh(&loaded.mesh, &loaded.attributes, profile, None)?;

            let ext = out_path
                .extension()
                .and_then(|s| s.to_str())
                .unwrap_or("stl");
            let validation = verify_roundtrip(RoundtripRequest {
                ext,
                original_mesh: &loaded.mesh,
                repaired_mesh: &result.mesh,
                repaired_attrs: &result.attributes,
                expected_report: &result.report.after,
                profile,
                was_noop: result.report.was_noop,
                limits: &limits,
            })?;

            if !validation.passed {
                return Err(format!(
                    "post-repair reimport validation failed: {}",
                    validation.errors.join("; ")
                )
                .into());
            }

            save_model(&out_path, &result.mesh, &result.attributes)?;

            if json {
                let payload = serde_json::json!({
                    "input": input,
                    "output": out_path,
                    "report": result.report,
                    "validation": validation,
                });
                println!("{}", serde_json::to_string_pretty(&payload)?);
            } else {
                println!("Input:   {}", input.display());
                println!("Output:  {}", out_path.display());
                println!("Profile: {}", profile.label());
                if result.report.was_noop {
                    println!("Result:  Clean input left unchanged (no-op).");
                } else {
                    for s in &result.report.kept_steps {
                        println!(
                            "  [KEPT] {} ({} -> {} faces): {}",
                            s.name, s.faces_before, s.faces_after, s.detail
                        );
                    }
                    for s in &result.report.rejected_steps {
                        println!("  [REJECTED] {}: {}", s.name, s.detail);
                    }
                }
                println!(
                    "After repair:\n{}",
                    validation.reimported_report.summary_text()
                );
                for note in &result.report.unresolved_notes {
                    println!("  [NOTE] {note}");
                }
                for w in &result.report.warnings {
                    println!("  [WARN] {w}");
                }
            }
        }
    }
    Ok(())
}

fn repair_threemf_cli(
    pkg: &ThreeMfPackage,
    input: &Path,
    out_path: &Path,
    profile: RepairProfile,
    json: bool,
    limits: &ResourceLimits,
) -> Result<(), Box<dyn std::error::Error>> {
    let out_ext = out_path
        .extension()
        .and_then(|s| s.to_str())
        .map(str::to_ascii_lowercase)
        .unwrap_or_default();
    if out_ext != "3mf" {
        return Err(
            "refusing to export a 3MF package to a non-3MF format (would strip colors/metadata); specify a .3mf output path"
                .into(),
        );
    }

    let mut part_results: Vec<RepairResult> = Vec::with_capacity(pkg.parts.len());
    for part in &pkg.parts {
        let res = repair_mesh(&part.mesh, &part.attributes, profile, None)?;
        part_results.push(res);
    }

    let repaired_refs: Vec<RepairedThreeMfPart<'_>> = pkg
        .parts
        .iter()
        .zip(part_results.iter())
        .map(|(orig, res)| RepairedThreeMfPart {
            archive_path: &orig.archive_path,
            object_id: &orig.object_id,
            was_noop: res.report.was_noop,
            mesh: &res.mesh,
            attributes: &res.attributes,
        })
        .collect();

    let expected_reports: Vec<DiagnosticReport> = part_results
        .iter()
        .map(|r| r.report.after.clone())
        .collect();

    let validation =
        verify_threemf_roundtrip(pkg, &repaired_refs, &expected_reports, profile, limits)?;

    if !validation.passed {
        return Err(format!(
            "post-repair 3MF reimport validation failed: {}",
            validation.errors.join("; ")
        )
        .into());
    }

    pkg.write_repaired_path(out_path, &repaired_refs)?;

    if json {
        let payload = serde_json::json!({
            "input": input,
            "output": out_path,
            "kind": match pkg.kind {
                ThreeMfKind::GeometryModel => "GeometryModel",
                ThreeMfKind::SlicerProject => "SlicerProject",
            },
            "profile": profile.label(),
            "parts": pkg.parts.iter().zip(part_results.iter()).map(|(p, r)| {
                serde_json::json!({
                    "object_id": p.object_id,
                    "object_name": p.object_name,
                    "report": r.report,
                })
            }).collect::<Vec<_>>(),
            "validation": validation,
        });
        println!("{}", serde_json::to_string_pretty(&payload)?);
    } else {
        println!("Input:   {} (3MF {:?})", input.display(), pkg.kind);
        println!("Output:  {}", out_path.display());
        println!("Profile: {}", profile.label());
        for (p, r) in pkg.parts.iter().zip(part_results.iter()) {
            println!(
                "\n-- Object {} ({}) --",
                p.object_id,
                p.object_name.as_deref().unwrap_or("unnamed")
            );
            if r.report.was_noop {
                println!("  No-op: clean geometry & original XML preserved unchanged.");
            } else {
                for s in &r.report.kept_steps {
                    println!(
                        "  [KEPT] {} ({} -> {} faces): {}",
                        s.name, s.faces_before, s.faces_after, s.detail
                    );
                }
                for s in &r.report.rejected_steps {
                    println!("  [REJECTED] {}: {}", s.name, s.detail);
                }
            }
            let a = &r.report.attribute_summary;
            println!(
                "  Attributes: {} exact, {} inherited, {} uncertain",
                a.exact_count, a.inherited_count, a.uncertain_count
            );
            for w in &r.report.warnings {
                println!("  [WARN] {w}");
            }
            for n in &r.report.unresolved_notes {
                println!("  [NOTE] {n}");
            }
        }
        println!("\nReimport validation: PASSED");
    }
    Ok(())
}

fn default_repaired_path(input: &Path) -> PathBuf {
    let parent = input.parent().unwrap_or_else(|| Path::new("."));
    let stem = input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("model");
    let ext = input.extension().and_then(|s| s.to_str()).unwrap_or("stl");
    parent.join(format!("{stem}_repaired.{ext}"))
}

fn paths_match(a: &Path, b: &Path) -> bool {
    if a == b {
        return true;
    }
    match (std::fs::canonicalize(a), std::fs::canonicalize(b)) {
        (Ok(ca), Ok(cb)) => ca == cb,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_core::TriangleMesh;
    use mesh_formats::write_stl_binary_path;
    use nalgebra::Point3;

    #[test]
    fn cli_repair_refuses_overwrite_and_repairs_hole() {
        let dir = tempfile::tempdir().expect("tempdir");
        let in_path = dir.path().join("hole.stl");
        let out_path = dir.path().join("hole_fixed.stl");

        let cube_verts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            Point3::new(100.0, 100.0, 0.0),
            Point3::new(0.0, 100.0, 0.0),
            Point3::new(0.0, 0.0, 100.0),
            Point3::new(100.0, 0.0, 100.0),
            Point3::new(100.0, 100.0, 100.0),
            Point3::new(0.0, 100.0, 100.0),
            Point3::new(49.0, 49.0, 100.0),
            Point3::new(51.0, 49.0, 100.0),
            Point3::new(51.0, 51.0, 100.0),
            Point3::new(49.0, 51.0, 100.0),
        ];
        // 100x100x100 block with a 2x2 mm square hole in the top face
        let cube_faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            [4, 5, 9],
            [4, 9, 8],
            [5, 6, 10],
            [5, 10, 9],
            [6, 7, 11],
            [6, 11, 10],
            [7, 4, 8],
            [7, 8, 11],
        ];
        write_stl_binary_path(&in_path, &TriangleMesh::from_parts(cube_verts, cube_faces))
            .expect("write");

        let overwrite_err = run(Cli {
            command: Command::Repair {
                input: in_path.clone(),
                profile: ProfileArg::Standard,
                output: Some(in_path.clone()),
                json: false,
            },
        })
        .expect_err("must refuse overwrite");
        assert!(overwrite_err.to_string().contains("overwrite"));

        run(Cli {
            command: Command::Repair {
                input: in_path,
                profile: ProfileArg::Standard,
                output: Some(out_path.clone()),
                json: false,
            },
        })
        .expect("repair");

        let reloaded = load_model(&out_path, &ResourceLimits::default()).expect("reload");
        assert!(analyze(&reloaded.mesh).is_watertight);
    }
}
