import numpy as np
import math

class TrackedObject:
    def __init__(self, obj_id, centroid):
        self.id = obj_id
        self.centroid = centroid  # (x, y)
        self.velocity = np.array([0.0, 0.0]) # (vx, vy)
        self.last_update_time = 0.0
        self.missed_frames = 0 # For deleting old objects

class EuclideanTracker:
    def __init__(self):
        self.tracked_objects = {}  # Dictionary {id: TrackedObject}
        self.next_id = 0
        self.max_missed_frames = 5  # Delete if not seen for 0.5s
        self.max_distance_threshold = 0.5 # Match if within 50cm

    def update(self, detected_centroids, current_time):
        # detected_centroids: List of (x, y) tuples from DBSCAN
        
        # If no objects tracked yet, register all as new
        if len(self.tracked_objects) == 0:
            for centroid in detected_centroids:
                self.register_object(centroid, current_time)
            return self.tracked_objects

        # Calculate distances between EXISTING objects and NEW centroids
        object_ids = list(self.tracked_objects.keys())
        object_centroids = [obj.centroid for obj in self.tracked_objects.values()]
        
        # If we have existing objects but no new detections, just mark missed
        if len(detected_centroids) == 0:
            for obj_id in object_ids:
                self.tracked_objects[obj_id].missed_frames += 1
            self.cleanup()
            return self.tracked_objects

        # --- MATCHING LOGIC (Greedy / Nearest Neighbor) ---
        # Calculate distance matrix (Rows: Existing, Cols: New)
        D = np.zeros((len(object_centroids), len(detected_centroids)))
        
        for i in range(len(object_centroids)):
            for j in range(len(detected_centroids)):
                dist = np.linalg.norm(np.array(object_centroids[i]) - np.array(detected_centroids[j]))
                D[i, j] = dist

        # Find pairs with min distance
        # rows = existing objects, cols = new detections
        used_rows = set()
        used_cols = set()

        # Simple Greedy assignment: Find smallest distance, assign, repeat
        while len(used_rows) < len(object_centroids) and len(used_cols) < len(detected_centroids):
            # Find min index in the whole matrix (that isn't used)
            min_val = np.inf
            r_min, c_min = -1, -1
            
            for r in range(D.shape[0]):
                if r in used_rows: continue
                for c in range(D.shape[1]):
                    if c in used_cols: continue
                    if D[r, c] < min_val:
                        min_val = D[r, c]
                        r_min, c_min = r, c
            
            # If closest match is too far, stop matching
            if min_val > self.max_distance_threshold:
                break

            # Update the object
            obj_id = object_ids[r_min]
            self.update_object(obj_id, detected_centroids[c_min], current_time)
            
            used_rows.add(r_min)
            used_cols.add(c_min)

        # Register any new centroids that weren't matched
        for c in range(len(detected_centroids)):
            if c not in used_cols:
                self.register_object(detected_centroids[c], current_time)

        # Mark unmatched existing objects as "missed"
        for r in range(len(object_centroids)):
            if r not in used_rows:
                obj_id = object_ids[r]
                self.tracked_objects[obj_id].missed_frames += 1

        self.cleanup()
        return self.tracked_objects

    def register_object(self, centroid, time):
        self.tracked_objects[self.next_id] = TrackedObject(self.next_id, centroid)
        self.tracked_objects[self.next_id].last_update_time = time
        self.next_id += 1

    def update_object(self, obj_id, new_centroid, time):
        obj = self.tracked_objects[obj_id]
        
        # Calculate Velocity: v = (p2 - p1) / dt
        dt = time - obj.last_update_time
        if dt > 0.001: # Avoid division by zero
            vx = (new_centroid[0] - obj.centroid[0]) / dt
            vy = (new_centroid[1] - obj.centroid[1]) / dt
            
            # Simple Low-Pass Filter (Smoothing) to reduce jitter
            alpha = 0.3
            obj.velocity[0] = (alpha * vx) + ((1-alpha) * obj.velocity[0])
            obj.velocity[1] = (alpha * vy) + ((1-alpha) * obj.velocity[1])

        obj.centroid = new_centroid
        obj.last_update_time = time
        obj.missed_frames = 0

    def cleanup(self):
        # Remove objects we haven't seen in a while
        to_delete = []
        for obj_id, obj in self.tracked_objects.items():
            if obj.missed_frames > self.max_missed_frames:
                to_delete.append(obj_id)
        for obj_id in to_delete:
            del self.tracked_objects[obj_id]