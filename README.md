# Robot-Aided-Drafter

parse_png(path: str | Path) -> List[Tuple[float,float,int]]
This function reads a PNG image from the specified path, extracts the skeleton of the image, and traverses its pixels to generate ordered (x,y,z) waypoints. Here, z=1 denotes a pen lift when transitioning between disconnected components in the image.

normalize(pts: Sequence[Tuple[float,float,int]]) -> List[Tuple[float,float,int]]
The normalize function adjusts a sequence of (x,y,z) waypoints to fit within the SCARA robot's workspace. It scales and translates the coordinates to ensure they fall within the defined X and Y ranges suitable for robot manipulation.

downsample_by_dist(pts: Sequence[Tuple[float,float,int]], min_dist: float) -> List[Tuple[float,float,int]]
This function reduces the density of waypoints in the provided sequence pts based on a minimum distance criterion min_dist. It retains points that are at least min_dist apart in the robot's workspace, ensuring a smoother and more manageable path.

ik_xy(x: float, y: float) -> Tuple[float,float]
ik_xy calculates the inverse kinematics for a given set of Cartesian coordinates (x, y). It computes the corresponding joint angles (t1, t2) in degrees using the SCARA robot's geometric parameters and offsets.

ik_path(pts: Sequence[Tuple[float,float,int]]) -> List[Tuple[float,float]]
This function generates a sequence of joint angles (t1, t2) in degrees for a path defined by the sequence of (x,y,z) waypoints. It applies the ik_xy function to each waypoint to convert Cartesian coordinates to joint angles suitable for robot control.

main()
The main function serves as the command-line interface entry point for the PNG to SCARA waypoints conversion process. It accepts arguments such as the input PNG file (png), output file (-o, --out), subsampling factor (--px-skip), and minimum distance between waypoints (--min-dist). It orchestrates the entire workflow by invoking the aforementioned functions to process the image and generate the desired output.
