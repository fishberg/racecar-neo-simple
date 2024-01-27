import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import PIL.Image
from io import BytesIO
import IPython.display
from IPython.display import clear_output
import pyrealsense2 as rs

class Camera():

    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                self.found_rgb = True
                break
        if not self.found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        try:
            self.pipeline.start(self.config)
        except:
            print('Camera not initialized')
        else:
            print('Camera initialized succesfully')
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
    
    def get_images(self):
        """
        Returns depth and color images as pair
        """
        # Get frameset of color and depth
        self.frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        self.aligned_frames = self.align.process(self.frames)

        # Get aligned frames
        self.aligned_depth_frame = self.aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        self.color_frame = self.aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not self.aligned_depth_frame or not self.color_frame:
            print("Frames not valid")

        self.depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        return self.depth_image, self.color_image

    def get_color_image(self):
        self.get_images()
        return self.color_image
    
    def get_depth_values(self):
        self.get_images()
        return self.depth_image

    def get_depth_image(self):
        self.get_images()
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return self.depth_colormap

    def get_both_frames(self):
        self.frames = self.pipeline.wait_for_frames()
        self.color_frame = self.frames.get_color_frame()
        self.depth_frame = self.frames.get_depth_frame()
        self.color_image = np.asanyarray(self.color_frame.get_data())
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_colormap_dim = self.color_image.shape
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        self.depth_colormap_dim = self.depth_colormap.shape

        if self.depth_colormap_dim != self.color_colormap_dim:
            self.resized_color_image = cv2.resize(self.color_image, dsize=(self.depth_colormap_dim[1], self.depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            self.images = np.hstack((self.resized_color_image, self.depth_colormap))
        else:
            self.images = np.hstack((self.color_image, self.depth_colormap))

        return self.images
    
    def show_depth_image(self, im_depth, im_color, clipping_distance=0, points=np.array([])):
        """
        Displays a color image in the Jupyter Notebook.

        Args:
            im_depth: The depth image to display.
            im_color: The RGB image to display.
            clipping_distance: The farthest depth to show in the RGB image in cm. Past this depth is shown as grey.
            points: A list of points in (pixel row, pixel column) format to show on the image colored dots.
        """
        # Remove background - Set pixels further than clipping_distance to grey
        self.clip_color = 153  # 153 = grey
        self.depth_image_3d = np.dstack((im_depth,im_depth,im_depth)) #depth image is 1 channel, color is 3 channels
        if clipping_distance > 0:
            self.bg_removed = np.where((self.depth_image_3d > clipping_distance) | (self.depth_image_3d <= 0), self.clip_color, im_color)
        else:
            self.bg_removed = np.where(self.depth_image_3d, im_color, im_color)

        # Draw a dot at each point in points
        for point in points:
            cv2.circle(self.bg_removed, (point[1], point[0]), 6, (0, 255, 0), -1)
        
        # Render images:
        #   depth align to color on left
        #   depth on right
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(im_depth, alpha=0.03), cv2.COLORMAP_JET)
        self.images = np.hstack((self.bg_removed, self.depth_colormap))        

        # Show the image with Matplotlib
#         clear_output(wait=True)
#         plt.figure(figsize = (16,12))
#         plt.imshow(self.images)
#         plt.show()
        self.f = BytesIO()
        PIL.Image.fromarray(self.images).save(self.f, 'jpeg')
        self.imgDisplay = IPython.display.Image(data=self.f.getvalue())
        self.display1.update(self.imgDisplay)
          
