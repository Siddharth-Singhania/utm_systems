"""
Virtual Drone Fleet Simulator
Simulates a fleet of drones sending telemetry to the UTM system
Each drone follows its assigned trajectory and reports position/status
"""

import asyncio
import aiohttp
import time
import math
import random
from typing import Optional, List
from models import Position, Telemetry, DroneStatus, Trajectory, Waypoint
import config
import pathfinding


class VirtualDrone:
    """Simulates a single drone with physics and battery simulation"""
    
    def __init__(self, drone_id: str, initial_position: Position):
        self.drone_id = drone_id
        self.position = initial_position
        self.velocity = (0.0, 0.0, 0.0)  # (vx, vy, vz) in m/s
        self.battery_level = 100.0
        self.status = DroneStatus.IDLE
        self.trajectory: Optional[Trajectory] = None
        self.current_waypoint_index = 0
        self.mission_start_time: Optional[float] = None
    
    def update_physics(self, dt: float):
        """
        Update drone position based on physics simulation
        
        Args:
            dt: Time delta in seconds
        """
        if self.status == DroneStatus.IDLE or not self.trajectory:
            return
        
        # Get current and next waypoint
        if self.current_waypoint_index >= len(self.trajectory.waypoints):
            self.status = DroneStatus.AT_DELIVERY
            self.velocity = (0.0, 0.0, 0.0)
            return
        
        current_waypoint = self.trajectory.waypoints[self.current_waypoint_index]
        target_pos = current_waypoint.position
        target_speed = current_waypoint.speed
        
        # Calculate distance to waypoint
        distance = pathfinding.distance_3d(
            self.position.latitude, self.position.longitude, self.position.altitude,
            target_pos.latitude, target_pos.longitude, target_pos.altitude
        )
        
        # Check if reached waypoint
        if distance < 5.0:  # Within 5 meters
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.trajectory.waypoints):
                self.status = DroneStatus.AT_DELIVERY
                self.velocity = (0.0, 0.0, 0.0)
                return
            return
        
        # Calculate direction to target
        # Simplified: Move in straight line to target
        lat_diff = target_pos.latitude - self.position.latitude
        lon_diff = target_pos.longitude - self.position.longitude
        alt_diff = target_pos.altitude - self.position.altitude
        
        # Normalize direction
        total_diff = math.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)
        if total_diff > 0:
            lat_dir = lat_diff / total_diff
            lon_dir = lon_diff / total_diff
            alt_dir = alt_diff / total_diff
        else:
            lat_dir = lon_dir = alt_dir = 0
        
        # Calculate movement (convert m/s to degrees per second)
        # Approximate: 1 degree latitude ≈ 111,320 meters
        meters_per_degree_lat = 111320.0
        meters_per_degree_lon = 111320.0 * math.cos(math.radians(self.position.latitude))
        
        lat_movement = (target_speed * lat_dir * dt) / meters_per_degree_lat
        lon_movement = (target_speed * lon_dir * dt) / meters_per_degree_lon
        alt_movement = target_speed * alt_dir * dt
        
        # Update position
        self.position.latitude += lat_movement
        self.position.longitude += lon_movement
        self.position.altitude += alt_movement
        
        # Update velocity
        self.velocity = (
            target_speed * lat_dir,
            target_speed * lon_dir,
            target_speed * alt_dir
        )
        
        # Consume battery (simplified model)
        power_consumption = config.DRONE_POWER_CONSUMPTION
        
        # Extra consumption when climbing
        if alt_diff > 0:
            power_consumption *= 1.5
        
        battery_consumed = (power_consumption * dt) / (config.DRONE_BATTERY_CAPACITY * 36)  # Convert Wh to %
        self.battery_level = max(0, self.battery_level - battery_consumed)
        
        # Check battery emergency
        if self.battery_level < 20.0:
            print(f"[{self.drone_id}] LOW BATTERY WARNING: {self.battery_level:.1f}%")
            if self.battery_level < 10.0:
                self.status = DroneStatus.EMERGENCY
    
    def assign_trajectory(self, trajectory: Trajectory):
        """Assign a new trajectory to follow"""
        self.trajectory = trajectory
        self.current_waypoint_index = 0
        self.status = DroneStatus.EN_ROUTE_PICKUP
        self.mission_start_time = time.time()
        print(f"[{self.drone_id}] Trajectory assigned: {len(trajectory.waypoints)} waypoints")
    
    def get_telemetry(self) -> Telemetry:
        """Get current telemetry data"""
        return Telemetry(
            drone_id=self.drone_id,
            position=self.position,
            velocity=self.velocity,
            battery_level=self.battery_level,
            status=self.status,
            timestamp=time.time()
        )


class FleetSimulator:
    """Manages a fleet of virtual drones"""
    
    def __init__(self, fleet_size: int, api_url: str):
        self.fleet_size = fleet_size
        self.api_url = api_url
        self.drones: List[VirtualDrone] = []
        self.running = False
    
    async def initialize_fleet(self):
        """Create and register virtual drones"""
        print(f"Initializing fleet of {self.fleet_size} virtual drones...")
        
        async with aiohttp.ClientSession() as session:
            for i in range(self.fleet_size):
                drone_id = f"drone_{i+1:03d}"
                
                # Random starting position within operational area
                bounds = config.OPERATIONAL_AREA
                lat = random.uniform(bounds['min_lat'], bounds['max_lat'])
                lon = random.uniform(bounds['min_lon'], bounds['max_lon'])
                alt = random.choice(config.ALTITUDE_LAYERS)
                
                initial_pos = Position(latitude=lat, longitude=lon, altitude=alt)
                
                # Create virtual drone
                drone = VirtualDrone(drone_id, initial_pos)
                self.drones.append(drone)
                
                # Register with UTM system
                try:
                    async with session.post(
                        f"{self.api_url}/api/drones/{drone_id}/register",
                        json={"latitude": lat, "longitude": lon, "altitude": alt}
                    ) as response:
                        if response.status == 200:
                            print(f"✓ {drone_id} registered at ({lat:.4f}, {lon:.4f}, {alt:.0f}m)")
                        else:
                            print(f"✗ Failed to register {drone_id}: {response.status}")
                except Exception as e:
                    print(f"✗ Error registering {drone_id}: {e}")
        
        print(f"Fleet initialization complete. {len(self.drones)} drones ready.")
    
    async def simulation_loop(self):
        """Main simulation loop - updates physics and sends telemetry"""
        self.running = True
        update_interval = 1.0 / config.TELEMETRY_UPDATE_RATE  # Convert Hz to seconds
        
        print(f"Starting simulation loop (update rate: {config.TELEMETRY_UPDATE_RATE} Hz)...")
        
        async with aiohttp.ClientSession() as session:
            while self.running:
                loop_start = time.time()
                
                # Update each drone
                for drone in self.drones:
                    # Update physics
                    drone.update_physics(update_interval)
                    
                    # Send telemetry to UTM system
                    telemetry = drone.get_telemetry()
                    
                    try:
                        async with session.post(
                            f"{self.api_url}/api/telemetry/update",
                            json=telemetry.model_dump(),
                            timeout=aiohttp.ClientTimeout(total=2)
                        ) as response:
                            if response.status != 200:
                                print(f"Telemetry update failed for {drone.drone_id}")
                    except asyncio.TimeoutError:
                        print(f"Timeout sending telemetry for {drone.drone_id}")
                    except Exception as e:
                        print(f"Error sending telemetry for {drone.drone_id}: {e}")
                
                # Maintain update rate
                elapsed = time.time() - loop_start
                sleep_time = max(0, update_interval - elapsed)
                await asyncio.sleep(sleep_time)
    
    async def mission_listener(self):
        """Listen for mission assignments via WebSocket"""
        print("Starting mission listener...")
        
        ws_url = self.api_url.replace('http://', 'ws://') + '/ws'
        
        async with aiohttp.ClientSession() as session:
            try:
                async with session.ws_connect(ws_url) as ws:
                    print(f"Connected to WebSocket: {ws_url}")
                    
                    async for msg in ws:
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            data = msg.json()
                            
                            if data.get('type') == 'mission_created':
                                mission_data = data.get('mission')
                                drone_id = mission_data.get('drone_id')
                                trajectory_data = mission_data.get('trajectory')
                                
                                if drone_id and trajectory_data:
                                    # Find drone and assign trajectory
                                    for drone in self.drones:
                                        if drone.drone_id == drone_id:
                                            trajectory = Trajectory(**trajectory_data)
                                            drone.assign_trajectory(trajectory)
                                            print(f"✓ Mission assigned to {drone_id}")
                                            break
                        
                        elif msg.type == aiohttp.WSMsgType.ERROR:
                            print(f"WebSocket error: {ws.exception()}")
                            break
            
            except Exception as e:
                print(f"Mission listener error: {e}")
                await asyncio.sleep(5)  # Wait before retry
    
    async def auto_mission_generator(self):
        """Automatically generate delivery missions for testing"""
        print("Starting auto-mission generator...")
        await asyncio.sleep(10)  # Wait for system to stabilize
        
        async with aiohttp.ClientSession() as session:
            mission_count = 0
            
            while self.running:
                # Generate random delivery mission
                bounds = config.OPERATIONAL_AREA
                
                pickup_lat = random.uniform(bounds['min_lat'], bounds['max_lat'])
                pickup_lon = random.uniform(bounds['min_lon'], bounds['max_lon'])
                pickup_alt = 0  # Ground level
                
                delivery_lat = random.uniform(bounds['min_lat'], bounds['max_lat'])
                delivery_lon = random.uniform(bounds['min_lon'], bounds['max_lon'])
                delivery_alt = 0  # Ground level
                
                request_data = {
                    "pickup": {
                        "latitude": pickup_lat,
                        "longitude": pickup_lon,
                        "altitude": pickup_alt
                    },
                    "delivery": {
                        "latitude": delivery_lat,
                        "longitude": delivery_lon,
                        "altitude": delivery_alt
                    }
                }
                
                try:
                    async with session.post(
                        f"{self.api_url}/api/delivery/request",
                        json=request_data
                    ) as response:
                        if response.status == 200:
                            result = await response.json()
                            mission_count += 1
                            print(f"✓ Mission {mission_count} created: {result.get('mission_id')}")
                        else:
                            error = await response.text()
                            print(f"✗ Mission creation failed: {error}")
                
                except Exception as e:
                    print(f"Error creating mission: {e}")
                
                # Wait before next mission
                await asyncio.sleep(config.MISSION_INTERVAL)
    
    async def run(self):
        """Run the complete simulation"""
        await self.initialize_fleet()
        
        # Run concurrent tasks
        await asyncio.gather(
            self.simulation_loop(),
            self.mission_listener(),
            self.auto_mission_generator()
        )
    
    def stop(self):
        """Stop the simulation"""
        self.running = False
        print("Stopping fleet simulator...")


async def main():
    """Main entry point"""
    print("=" * 60)
    print("UTM VIRTUAL DRONE FLEET SIMULATOR")
    print("=" * 60)
    
    api_url = f"http://localhost:{config.API_PORT}"
    
    # Wait for backend to be ready
    print(f"Waiting for UTM backend at {api_url}...")
    await asyncio.sleep(3)
    
    fleet = FleetSimulator(
        fleet_size=config.SIMULATION_FLEET_SIZE,
        api_url=api_url
    )
    
    try:
        await fleet.run()
    except KeyboardInterrupt:
        print("\nShutdown signal received...")
        fleet.stop()


if __name__ == "__main__":
    asyncio.run(main())
