from enum import Enum
from datetime import datetime
from pydantic import BaseModel, Field
from typing import List, Optional

# Data models
class ConnectionType(str, Enum):
    SERIAL = "serial"
    UDP = "udp"
    TCP = "tcp"

class ConnectionRequest(BaseModel):
    connection_type: ConnectionType
    device: str = Field(..., description="Serial device, UDP address:port, or TCP address:port")
    baud: Optional[int] = Field(57600, description="Baud rate for serial connections")
    timeout: Optional[float] = Field(1.0, description="Connection timeout in seconds")
    name: Optional[str] = Field(None, description="User-friendly name for this connection")

class CommandRequest(BaseModel):
    command_id: int
    params: List[float] = Field(default_factory=lambda: [0, 0, 0, 0, 0, 0, 0])
    target_system: Optional[int] = None
    target_component: Optional[int] = None
    confirmation: Optional[int] = Field(0, description="Confirmation parameter")

class WaypointRequest(BaseModel):
    lat: float = Field(..., description="Latitude in degrees")
    lon: float = Field(..., description="Longitude in degrees")
    alt: float = Field(..., description="Altitude in meters (relative to home)")
    seq: int = Field(..., description="Waypoint sequence number")

class WaypointResponse(BaseModel):
    """Use for refelect waypoint struct"""
    id: int
    mission_id: int
    lat: float
    lon: float
    alt: float
    seq: int
    created_at: datetime
    updated_at: Optional[datetime] = None
    
    class Config:
        from_attributes = True

class MissionRequest(BaseModel):
    waypoints: List[WaypointRequest]
    clear_existing: bool = Field(True, description="Clear existing mission before uploading")

class ConnectionStatus(BaseModel):
    id: str
    name: str
    connection_type: ConnectionType
    device: str
    connected: bool
    vehicle_type: Optional[str] = None
    system_id: Optional[int] = None
    last_heartbeat: Optional[float] = None
    created_at: float

class MissionBase(BaseModel):
    pass

class MissionCreate(MissionBase):
    waypoints: List[WaypointRequest]

class MissionUpdate(MissionBase):
    waypoints: Optional[List[WaypointRequest]] = None

class MissionSummary(MissionBase):
    """Schema for returning brief mission data without waypoints"""
    id: int
    waypoint_count: int
    created_at: datetime
    updated_at: Optional[datetime] = None
    
    class Config:
        from_attributes = True

class MissionDetail(MissionBase):
    id: int
    waypoints: List[WaypointResponse]
    created_at: datetime
    updated_at: Optional[datetime] = None
    
    class Config:
        from_attributes = True