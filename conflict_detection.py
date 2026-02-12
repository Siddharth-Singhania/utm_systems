"""
Conflict Detection and Resolution Module
Detects 4D conflicts and applies resolution strategies:
1. Speed adjustment (primary method - more energy efficient)
2. Altitude adjustment (secondary method)
3. Path re-planning (last resort)
"""

from typing import List, Tuple, Optional
from models import Trajectory, Waypoint, Position4D, ConflictAlert
import config
import pathfinding
import time
import uuid


class ConflictDetector:
    """Manages conflict detection between drone trajectories"""
    
    def __init__(self):
        self.active_conflicts: List[ConflictAlert] = []
    
    def detect_conflicts(self, trajectories: dict) -> List[ConflictAlert]:
        """
        Detect all pairwise conflicts in active trajectories
        
        Args:
            trajectories: Dict of {drone_id: Trajectory}
        
        Returns:
            List of detected conflicts
        """
        conflicts = []
        drone_ids = list(trajectories.keys())
        
        # Check all pairs of drones
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                drone_1 = drone_ids[i]
                drone_2 = drone_ids[j]
                
                conflict = self.check_trajectory_conflict(
                    drone_1, trajectories[drone_1],
                    drone_2, trajectories[drone_2]
                )
                
                if conflict:
                    conflicts.append(conflict)
        
        self.active_conflicts = conflicts
        return conflicts
    
    def check_trajectory_conflict(self, drone_1_id: str, traj_1: Trajectory,
                                  drone_2_id: str, traj_2: Trajectory) -> Optional[ConflictAlert]:
        """
        Check if two trajectories conflict in 4D space-time
        
        A conflict occurs when drones are within separation minima at the same time:
        - Horizontal separation < HORIZONTAL_SEPARATION
        - Vertical separation < VERTICAL_SEPARATION
        - Time overlap within TIME_RESOLUTION
        
        Args:
            drone_1_id, traj_1: First drone and trajectory
            drone_2_id, traj_2: Second drone and trajectory
        
        Returns:
            ConflictAlert if conflict detected, None otherwise
        """
        # Sample both trajectories at regular time intervals
        t_start = max(traj_1.waypoints[0].eta, traj_2.waypoints[0].eta)
        t_end = min(traj_1.waypoints[-1].eta, traj_2.waypoints[-1].eta)
        
        if t_start >= t_end:
            return None  # No temporal overlap
        
        # Check conflicts at discrete time steps
        current_time = t_start
        while current_time <= t_end:
            pos_1 = self.interpolate_position(traj_1, current_time)
            pos_2 = self.interpolate_position(traj_2, current_time)
            
            if pos_1 and pos_2:
                horizontal_dist = pathfinding.haversine_distance(
                    pos_1.latitude, pos_1.longitude,
                    pos_2.latitude, pos_2.longitude
                )
                vertical_dist = abs(pos_1.altitude - pos_2.altitude)
                
                # Check if separation minima violated
                if (horizontal_dist < config.HORIZONTAL_SEPARATION and
                    vertical_dist < config.VERTICAL_SEPARATION):
                    
                    # Conflict detected!
                    return ConflictAlert(
                        conflict_id=str(uuid.uuid4()),
                        drone_1_id=drone_1_id,
                        drone_2_id=drone_2_id,
                        conflict_position=pos_1,
                        conflict_time=current_time,
                        severity=self.assess_severity(horizontal_dist, vertical_dist),
                        resolution_action=None
                    )
            
            current_time += config.TIME_RESOLUTION
        
        return None
    
    def interpolate_position(self, trajectory: Trajectory, time: float) -> Optional[Position4D]:
        """
        Interpolate drone position at a specific time
        
        Args:
            trajectory: Drone trajectory
            time: Query time (Unix timestamp)
        
        Returns:
            Interpolated position or None if time is outside trajectory
        """
        waypoints = trajectory.waypoints
        
        # Find surrounding waypoints
        for i in range(len(waypoints) - 1):
            if waypoints[i].eta <= time <= waypoints[i + 1].eta:
                # Linear interpolation
                w1 = waypoints[i]
                w2 = waypoints[i + 1]
                
                t_ratio = (time - w1.eta) / (w2.eta - w1.eta) if w2.eta != w1.eta else 0
                
                lat = w1.position.latitude + t_ratio * (w2.position.latitude - w1.position.latitude)
                lon = w1.position.longitude + t_ratio * (w2.position.longitude - w1.position.longitude)
                alt = w1.position.altitude + t_ratio * (w2.position.altitude - w1.position.altitude)
                
                return Position4D(
                    latitude=lat,
                    longitude=lon,
                    altitude=alt,
                    timestamp=time
                )
        
        return None
    
    def assess_severity(self, horizontal_dist: float, vertical_dist: float) -> str:
        """
        Assess conflict severity based on separation distances
        
        Returns:
            "critical", "warning", or "minor"
        """
        if horizontal_dist < config.HORIZONTAL_SEPARATION / 2:
            return "critical"
        elif horizontal_dist < config.HORIZONTAL_SEPARATION * 0.75:
            return "warning"
        else:
            return "minor"


class ConflictResolver:
    """Resolves detected conflicts by modifying trajectories"""
    
    def __init__(self):
        self.conflict_detector = ConflictDetector()
    
    def resolve_conflict(self, conflict: ConflictAlert, 
                        traj_1: Trajectory, traj_2: Trajectory) -> Tuple[Trajectory, str]:
        """
        Resolve a conflict by modifying one trajectory
        
        Strategy priority:
        1. Speed adjustment (most energy efficient)
        2. Altitude change (if speed adjustment insufficient)
        3. Path re-planning (if other methods fail)
        
        Args:
            conflict: Detected conflict
            traj_1: First drone's trajectory (will be modified)
            traj_2: Second drone's trajectory (reference)
        
        Returns:
            (modified_trajectory, resolution_method)
        """
        # Try speed adjustment first
        modified_traj = self.adjust_speed(traj_1, conflict)
        if modified_traj and not self.trajectories_conflict(modified_traj, traj_2):
            return modified_traj, "speed_adjustment"
        
        # Try altitude adjustment
        modified_traj = self.adjust_altitude(traj_1, conflict)
        if modified_traj and not self.trajectories_conflict(modified_traj, traj_2):
            return modified_traj, "altitude_change"
        
        # Last resort: suggest re-planning
        return traj_1, "replan_required"
    
    def adjust_speed(self, trajectory: Trajectory, conflict: ConflictAlert) -> Optional[Trajectory]:
        """
        Adjust drone speed to avoid conflict while maintaining path
        
        Strategy: Slow down before conflict point to arrive after the other drone passes
        
        Args:
            trajectory: Original trajectory
            conflict: Conflict to resolve
        
        Returns:
            Modified trajectory with adjusted speeds
        """
        new_waypoints = []
        time_shift = config.TIME_RESOLUTION * 3  # Delay arrival by 15 seconds
        
        for i, waypoint in enumerate(trajectory.waypoints):
            if waypoint.eta < conflict.conflict_time:
                # Before conflict - reduce speed gradually
                reduction_factor = 0.7  # 30% speed reduction
                new_speed = max(waypoint.speed * reduction_factor, config.DRONE_MIN_SPEED)
                
                # Recalculate ETA based on new speed
                if i > 0:
                    prev_wp = new_waypoints[i - 1]
                    distance = pathfinding.distance_3d(
                        prev_wp.position.latitude, prev_wp.position.longitude, prev_wp.position.altitude,
                        waypoint.position.latitude, waypoint.position.longitude, waypoint.position.altitude
                    )
                    new_eta = prev_wp.eta + (distance / new_speed)
                else:
                    new_eta = waypoint.eta
                
                new_waypoints.append(Waypoint(
                    position=waypoint.position,
                    eta=new_eta,
                    speed=new_speed,
                    heading=waypoint.heading
                ))
            else:
                # After conflict - resume normal speed
                if i > 0:
                    prev_wp = new_waypoints[i - 1]
                    distance = pathfinding.distance_3d(
                        prev_wp.position.latitude, prev_wp.position.longitude, prev_wp.position.altitude,
                        waypoint.position.latitude, waypoint.position.longitude, waypoint.position.altitude
                    )
                    new_eta = prev_wp.eta + (distance / config.DRONE_CRUISE_SPEED)
                else:
                    new_eta = waypoint.eta + time_shift
                
                new_waypoints.append(Waypoint(
                    position=waypoint.position,
                    eta=new_eta,
                    speed=config.DRONE_CRUISE_SPEED,
                    heading=waypoint.heading
                ))
        
        # Recalculate trajectory metadata
        total_time = new_waypoints[-1].eta - new_waypoints[0].eta
        battery_usage = (total_time * config.DRONE_POWER_CONSUMPTION / 
                        config.DRONE_BATTERY_CAPACITY * 100)
        
        return Trajectory(
            waypoints=new_waypoints,
            total_distance=trajectory.total_distance,
            total_time=total_time,
            estimated_battery_usage=battery_usage
        )
    
    def adjust_altitude(self, trajectory: Trajectory, conflict: ConflictAlert) -> Optional[Trajectory]:
        """
        Adjust altitude to create vertical separation
        
        Args:
            trajectory: Original trajectory
            conflict: Conflict to resolve
        
        Returns:
            Modified trajectory with different altitude
        """
        new_waypoints = []
        altitude_shift = config.VERTICAL_SEPARATION + 5  # Add 5m buffer
        
        for waypoint in trajectory.waypoints:
            # Shift altitude up (respecting max altitude limit)
            new_altitude = min(
                waypoint.position.altitude + altitude_shift,
                config.DRONE_MAX_ALTITUDE
            )
            
            new_position = Position(
                latitude=waypoint.position.latitude,
                longitude=waypoint.position.longitude,
                altitude=new_altitude
            )
            
            new_waypoints.append(Waypoint(
                position=new_position,
                eta=waypoint.eta,
                speed=waypoint.speed,
                heading=waypoint.heading
            ))
        
        return Trajectory(
            waypoints=new_waypoints,
            total_distance=trajectory.total_distance,
            total_time=trajectory.total_time,
            estimated_battery_usage=trajectory.estimated_battery_usage
        )
    
    def trajectories_conflict(self, traj_1: Trajectory, traj_2: Trajectory) -> bool:
        """
        Quick check if two trajectories conflict
        
        Returns:
            True if conflict exists, False otherwise
        """
        conflict = self.conflict_detector.check_trajectory_conflict(
            "test_1", traj_1, "test_2", traj_2
        )
        return conflict is not None
