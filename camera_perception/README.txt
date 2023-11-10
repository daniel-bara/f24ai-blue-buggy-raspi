
Camera_Perception (folder)
	INPUT  - Camera view
	OUTPUT - Cone types and locations

	Code exists to use either Tensor Flow (TF) or Computer Vision (CV) for recognition
		Tensor Flow remains unused throughout

__init__.py
	Initialises the three classes used for camera perception
	The tensorflow class is commented out and was unused in F21

camera_perception_abc.py
	Creates abstract base class (a metaclass which defines behaviour of classess within it)
	This is to define how the TF and CV sub-classes behave

	def get_cones(self,frame)
		It simply defines the function to return the location and type of cones

cv_camera_perception.py
	Creates the child class CVCameraPerception
	
	def __init__(self)
		Uses the 'super' method to allow the child class to access methods from parent class
		I'm unsure what the '__init__()' does
			Does this re-run the __init__.py file?

	def get_cones(self, frame)
		Smooths image
		Converts to hsv (better colour format for recognition)
		Finds all areas with blue (within pre-defined BLUE_RANGE)
		Finds all areas with yellow (within pre-defined YELLOW_RANGE)
		Performs a closed morphology to remove the different colours within the cone
			Is this to remove the horizontal line in the cone?
		Draws contours around cone shapes (contours being the edges)
		Draws these contour shapes onto the original frame image
		Analyse each contour
			If the area is less than area_threshold
				It isn't a cone
				CONSIDER A MAX THRESHOLD TOO
			Approximate contour to polygon (hopefully triangle)
				CONSIDER MAKING THIS A REQUIREMENT TO DEFINE AS CONE
			x = midpoint horizontally
			y = highest co-ordinate

		Return
			The edited frame (smoothed, drawn contours)
			b_points
				x, y, area
			y_points
				x, y, area