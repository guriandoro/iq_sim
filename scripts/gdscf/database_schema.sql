DROP TABLE IF EXISTS drone_camera;
DROP TABLE IF EXISTS drone_camera_blob;

CREATE TABLE drone_camera (
  drone_id int, 
  mission_id int, 
  secs int, 
  nsecs int, 
  cell_R int, 
  cell_G int, 
  cell_B int
);

CREATE INDEX idx_dc_1 ON drone_camera (drone_id, mission_id);

CREATE TABLE drone_camera_blob (
  drone_id int, 
  mission_id int, 
  secs int, 
  nsecs int, 
  latitude double precision,
  longitude double precision,
  altitude double precision,
  image bytea);

CREATE INDEX idx_dcb_1 ON drone_camera_blob (drone_id, mission_id);

