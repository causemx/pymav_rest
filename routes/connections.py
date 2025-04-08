import asyncio
import time
import uuid
from fastapi import APIRouter, HTTPException, Query, Depends
from pymavlink import mavutil
from typing import Optional, List
from loguru import logger

import config
from models.schemas import ConnectionRequest, ConnectionStatus
from utils.mavlink_helpers import get_mavlink_connection

router = APIRouter(prefix="/connections")

async def update_connection_status(connection_id):
    """Background task to update connection status"""
    connection_data = config.mavlink_connections.get(connection_id)
    if not connection_data:
        return
    
    mavlink_connection = connection_data["connection"]
    status = connection_data["status"]
    
    while status["connected"]:
        try:
            # Try to get heartbeat
            msg = mavlink_connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                status["last_heartbeat"] = time.time()
                if not status["vehicle_type"]:
                    status["vehicle_type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name
                if not status["system_id"]:
                    status["system_id"] = mavlink_connection.target_system
            
            # Check if connection is still alive
            if status["last_heartbeat"] and time.time() - status["last_heartbeat"] > 5:
                status["connected"] = False
                break
                
        except Exception as e:
            logger.error(f"Error updating connection status for {connection_id}: {e}")
            status["connected"] = False
            break
            
        await asyncio.sleep(1)

@router.get("/")
async def list_connections():
    """
    List all MAVLink connections.
    Returns details about all vehicle connections including connection status, type, and vehicle info.
    """
    connection_list = []
    
    for conn_id, conn_data in config.mavlink_connections.items():
        status = conn_data["status"]
        connection_list.append({
            "id": conn_id,
            "name": status["name"],
            "connection_type": status["connection_type"],
            "device": status["device"],
            "connected": status["connected"],
            "vehicle_type": status["vehicle_type"],
            "system_id": status["system_id"],
            "last_heartbeat": status["last_heartbeat"],
            "is_current": conn_id == config.current_connection_id,
            "created_at": status["created_at"]
        })
    
    return {
        "total": len(connection_list),
        "connections": connection_list,
        "current_connection": config.current_connection_id
    }

@router.get("/status")
async def get_status(connection_id: Optional[str] = None):
    if connection_id:
        if connection_id not in config.mavlink_connections:
            raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
        return config.mavlink_connections[connection_id]["status"]
    
    if config.current_connection_id:
        return config.mavlink_connections[config.current_connection_id]["status"]
    
    raise HTTPException(status_code=503, detail="No active MAVLink connection")

@router.post("/connect")
async def connect(connection_req: ConnectionRequest):
    try:
        # Create connection string based on connection type
        if connection_req.connection_type == "SERIAL":
            conn_str = f"{connection_req.connection_type.value}:{connection_req.device}:{connection_req.baud}"
        else:
            conn_str = f"{connection_req.connection_type.value}:{connection_req.device}"
        
        # Establish connection
        mavlink_connection = mavutil.mavlink_connection(conn_str, autoreconnect=True, timeout=connection_req.timeout)
        
        # Wait for heartbeat
        msg = mavlink_connection.wait_heartbeat(timeout=5)
        if not msg:
            raise HTTPException(status_code=504, detail="No heartbeat received after 5 seconds")
        
        # Generate unique ID for this connection
        connection_id = str(uuid.uuid4())
        
        # Create connection name if not provided
        connection_name = connection_req.name
        if not connection_name:
            connection_name = f"{connection_req.connection_type.value}-{connection_req.device}"
        
        # Create connection status
        connection_status = {
            "id": connection_id,
            "name": connection_name,
            "connection_type": connection_req.connection_type,
            "device": connection_req.device,
            "connected": True,
            "vehicle_type": mavutil.mavlink.enums['MAV_TYPE'][msg.type].name,
            "system_id": mavlink_connection.target_system,
            "last_heartbeat": time.time(),
            "created_at": time.time()
        }
        
        # Store connection in registry
        config.mavlink_connections[connection_id] = {
            "connection": mavlink_connection,
            "status": connection_status
        }
        
        # Set as current connection
        config.current_connection_id = connection_id
        
        # Start background task to update connection status
        asyncio.create_task(update_connection_status(connection_id))
        
        return {
            "status": "connected", 
            "connection_id": connection_id,
            "details": connection_status
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Connection failed: {str(e)}")

@router.post("/set_active_connection/{connection_id}")
async def set_active_connection(connection_id: str):
    if connection_id not in config.mavlink_connections:
        raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
    
    if not config.mavlink_connections[connection_id]["status"]["connected"]:
        raise HTTPException(status_code=400, detail=f"Connection {connection_id} is not active")
    
    config.current_connection_id = connection_id
    
    return {
        "status": "success",
        "message": f"Connection {connection_id} is now active",
        "connection": config.mavlink_connections[connection_id]["status"]
    }

@router.post("/disconnect")
async def disconnect(connection_id: Optional[str] = None):
    # If no connection_id provided, use current connection
    if connection_id is None:
        connection_id = config.current_connection_id
    
    if connection_id is None:
        raise HTTPException(status_code=400, detail="No connection specified")
    
    if connection_id not in config.mavlink_connections:
        raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
    
    connection_data = config.mavlink_connections[connection_id]
    connection_data["connection"].close()
    connection_data["status"]["connected"] = False
    del config.mavlink_connections[connection_id]

    # If disconnecting current connection, set current to None
    if connection_id == config.current_connection_id:
        config.current_connection_id = None
        
        # Try to set another active connection as current
        for conn_id, conn_data in config.mavlink_connections.items():
            if conn_id != connection_id and conn_data["status"]["connected"]:
                config.current_connection_id = conn_id
                break
    
    return {
        "status": "disconnected",
        "connection_id": connection_id,
        "current_connection": config.current_connection_id
    }

@router.delete("/{connection_id}")
async def delete_connection(connection_id: str):
    if connection_id not in config.mavlink_connections:
        raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
    
    # Disconnect if connected
    connection_data = config.mavlink_connections[connection_id]
    if connection_data["status"]["connected"]:
        connection_data["connection"].close()
    
    # Remove from registry
    del config.mavlink_connections[connection_id]
    
    # If deleting current connection, set current to None
    if connection_id == config.current_connection_id:
        config.current_connection_id = None
        
        # Try to set another active connection as current
        for conn_id, conn_data in config.mavlink_connections.items():
            if conn_data["status"]["connected"]:
                config.current_connection_id = conn_id
                break
    
    return {
        "status": "deleted",
        "connection_id": connection_id,
        "current_connection": config.current_connection_id
    }