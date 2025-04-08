# MAVLink API Server

A RESTful API server for interacting with MAVLink-enabled vehicles.

## Project Structure

```
mavlink_api/
├── app.py                  # Main application setup
├── requirements.txt        # Dependencies
├── config.py               # Configuration variables
├── models/                 # Pydantic models
│   └── __init__.py
│   └── schemas.py          # Data models
├── routes/                 # API routes
│   └── __init__.py
│   └── connections.py      # Connection endpoints
│   └── vehicle.py          # Vehicle control endpoints
│   └── mission.py          # Mission endpoints
└── utils/                  # Utility functions
    └── __init__.py
    └── mavlink_helpers.py  # Helper functions for MAVLink
```

## Installation

1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt`
3. Run the server: `python app.py`

## API Overview

The API is organized into three main sections:

1. **Connections** - Manage connections to MAVLink vehicles
2. **Vehicle** - Control vehicle state (arm, disarm, takeoff, land, etc.)
3. **Mission** - Upload and manage waypoint missions

### Connection Management

- `GET /connections` - List all connections
- `GET /connections/status` - Get connection status
- `POST /connections/connect` - Connect to a vehicle
- `POST /connections/set_active_connection/{connection_id}` - Set active connection
- `POST /connections/disconnect` - Disconnect from a vehicle
- `DELETE /connections/{connection_id}` - Delete a connection

### Vehicle Control

- `GET /vehicle/parameters` - Get vehicle parameters
- `GET /vehicle/attitude` - Get vehicle attitude
- `GET /vehicle/location` - Get vehicle location
- `POST /vehicle/arm` - Arm the vehicle
- `POST /vehicle/disarm` - Disarm the vehicle
- `POST /vehicle/takeoff` - Takeoff
- `POST /vehicle/land` - Land
- `POST /vehicle/return_to_launch` - Return to launch
- `POST /vehicle/goto` - Go to position
- `POST /vehicle/command` - Send custom command

### Mission Management

- `POST /mission/upload` - Upload a mission
- `GET /mission/current` - Get current mission
- `POST /mission/start` - Start mission

## Usage Examples

### Connect to a Vehicle

```bash
curl -X POST "http://localhost:8000/connections/connect" \
  -H "Content-Type: application/json" \
  -d '{"connection_type": "udp", "device": "127.0.0.1:14550"}'
```

### Arm the Vehicle

```bash
curl -X POST "http://localhost:8000/vehicle/arm"
```

### Upload a Mission

```bash
curl -X POST "http://localhost:8000/mission/upload" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 47.397742, "lon": 8.545594, "alt": 50, "seq": 0},
      {"lat": 47.398242, "lon": 8.546594, "alt": 50, "seq": 1}
    ],
    "clear_existing": true
  }'
```