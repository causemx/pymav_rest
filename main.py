import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from loguru import logger

from routes import connections, vehicle, mission

# Create FastAPI application
app = FastAPI(
    title="MAVLink API Server",
    description="RESTful API for interacting with MAVLink-enabled vehicles",
    version="1.0.0"
)

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global registry for all connections
mavlink_connections = {}
current_connection_id = None

@app.get("/")
async def root():
    return {"message": "MAVLink API Server is running!"}

# Include all routers
app.include_router(connections.router, tags=["connections"])
app.include_router(vehicle.router, tags=["vehicle"])
app.include_router(mission.router, tags=["mission"])


if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)