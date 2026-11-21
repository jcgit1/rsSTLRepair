//! Local repair HTTP service. Binds to 127.0.0.1:8787.

use mesh_web::{ServiceLimits, router};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr =
        std::env::var("MESH_REPAIR_WEB_ADDR").unwrap_or_else(|_| "127.0.0.1:8787".to_string());
    let listener = tokio::net::TcpListener::bind(&addr).await?;
    eprintln!("mesh-repair-web listening on http://{addr}");
    axum::serve(listener, router(ServiceLimits::default())).await?;
    Ok(())
}
