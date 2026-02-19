# Message & Event System

## Overview

Services communicate through a typed event system. Events are broadcast by ServiceManager to all registered services.

## ServiceEvent

Services emit and receive ServiceEvent objects containing:
- **event_source**: ServiceIdentifier (which service sent the event)
- **event_type**: Type of event
- **event_data**: std::variant with event-specific payload

## Event Types

| Event | Source | Description |
|-------|--------|-------------|
| SERVICE_INIT | Any | Service initialized |
| SERVICE_ACTIVE | Any | Service started processing |
| SERVICE_INACTIVE | Any | Service completed processing |
| SERVICE_LOG_UPDATED | Sensors | New log entry written |
| GNSS_ON | GPS | GNSS acquisition started |
| GNSS_OFF | GPS | GNSS acquisition completed |

## Event Flow

1. Service calls service_complete() or directly emits event
2. ServiceManager::notify_peer_event() broadcasts to all services
3. Each service's notify_peer_event() checks if event is relevant
4. Service may trigger its own action in response

## ServiceIdentifier Enum

| Value | Service |
|-------|---------|
| SERVICE_MANAGER | ServiceManager itself |
| GNSS_SENSOR | GPSService |
| UW_SENSOR | Underwater detection |
| ALS_SENSOR | Ambient light |
| AXL_SENSOR | Accelerometer |
| CDT_SENSOR | Conductivity-Depth-Temp |
| PH_SENSOR | pH |
| PRESSURE_SENSOR | Pressure |
| SEA_TEMP_SENSOR | Sea temperature |
| THERMISTOR_SENSOR | Thermistor |
| CAM_SENSOR | Camera |
| ARGOS_TX_SERVICE | Argos TX |
| ARGOS_RX_SERVICE | Argos RX |
| DIVE_MODE | Dive mode |
| MEMORY_MONITOR | Memory monitor |

## Underwater State Notification

A special broadcast path exists for underwater state changes:
```cpp
ServiceManager::notify_underwater_state(bool is_underwater);
```
This notifies all services simultaneously, allowing them to:
- Defer scheduling when submerged (if !service_is_usable_underwater())
- Trigger GNSS acquisition on surfacing (GNSS_TRIGGER_ON_SURFACED)
- Start/stop dive mode tracking
