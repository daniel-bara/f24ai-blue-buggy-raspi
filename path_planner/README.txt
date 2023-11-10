
path_planner (folder)
	Creates a path of coordinates for the buggy to follow

	Option to use two different path planning strategies. Wall centre line and point centre line


-----

__init__.py
	Initialises the three classes used for path planning
	abc being the parent class

-----

path_planner_abc.py
	Parent class for the two path planning classes.
	Defines the overall inputs and outputs

-----

wall_centre_line.py
	
	Looks at blue cones and creates 'wall'
	Looks at yellow cones and creates 'wall'
	Returns centreline between both walls

	def __init__
		super method allows to call methods from parent class
		'blue_wall' and 'yellow_wall' are declared

	---

	def path generator
		INPUT  - blue_cones, yellow_cones, vehicle_vectors
		OUTPUT - Array of centre points, metadata (FOR REVIEW?)

		Create inclusion box
			For XXX
		
		If no cones are in sight, return nothing

-----

point_centre_line.py
	THIS CLASS WAS DEEMED PROBLEMATIC, COMPLEX AND RELIANT ON ACCURATE SENSOR VALUES AND IS NOT USED IN THIS PROJECT
	
	def __init__
		Super method allows to call methods from parent class
		Sets radius to 25 (RENAME)
			Radius is later known as ang * distance
			The larger radius is, the further cones can be to be considered for path planning

	---

	def path_generator
		INPUT  - blue_cones, yellow_cones, vehicle_vectors
		OUTPUT - Array of centre points

		The vehicle vectors consist of: position, forward, right
			HOW DOES THE VEHICLE KNOW ITS POSITION?
				GPS?
			WHAT IS VEHICLE FORWARD/RIGHT?

		If no cones are in sight, return nothing

		Filters for relevant cones using the select_included_coords function
			Relevant cones are (those either close to or infront of vehicle)

		pair each blue cone with nearest yellow cone + each yellow with nearest blue
			Using the find_nearest function
				A pair is dismissed if distance > 25/2
					WHY THIS DISTANCE?

		Calculate the centrepoint between each pair
			If centrepoint is too close to a cone, skip
				REVIEW THIS LINE OF CODE, 'if any(...)'
					im not sure how it works

	---

	def select_included_coords
		INPUT  - all cone coords
		OUTPUT - some cone coords
		Used in path_generator function to filter cones

		if cone is close
			include it

		if cone is far but kind of infront
			include it

		Exclude the rest

	---
	
	def find_nearest
		INPUT  - blue cone coords, yellow cone coords
		OUTPUT - pairs of yellow & blue cones
		Used in path_generator to make cone pairs

		This section assumes the two nearst cones of differing coulours make up a pair
			THIS ISN'T ALWAYS TRUE BUT COULD WORK FOR US

-----

