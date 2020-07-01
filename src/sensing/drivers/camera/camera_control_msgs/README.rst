====
**Message package for camera drivers**
====
**developed by Magazino GmbH, using the pylon Software Camera Suite by Basler AG**

.. image:: https://raw.githubusercontent.com/magazino/pylon_camera/indigo-devel/wiki_imgs/logos.png
   :width: 130 %
   :align: center

This package offers many service and action definitions to control a camera.



******
**Services**
******
- **SetBinning**
  Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger 'super-pixels'. It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling). Calling this service with target binning values will change the binning entry in the published camera_info_msg of the camera.

- **SetBrightness**
  The target brightness, which is average intensity values of the images. It depends the exposure time as well as the gain setting.
  The brightness_continuous flag controls the auto brightness function. If it is set to false, the given brightness will only be reached once.
  Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously,
  trying to adapt to changing light conditions. The 'brightness_contunuous' mode is is only possible for values in the possible auto range of the camera which is e.g. [50 - 205].
  If the camera should try reach or keep the desired brightness, hence adapting to changing light conditions, at least one of the following flags **MUST** be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. 'exposure_auto' will adapt the exposure time to reach the brightness, wheras 'gain_auto' does so by adapting the gain.

- **SetExposure**
  The target exposure time measured in microseconds. If the limits were exceeded, the desired exposure time will be truncated.

- **SetGain**
  The target gain in percent of the maximal value the camera supports.

- **SetGamma**
  The target gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).

- **SetSleeping**
  If the camera runs in topic mode (continuously publishing images over the topics respecting the desired frame_rate) this service offers the posibillity to pause the image acquisition. To restart the grabbing, this service should be called again with set_sleeping set to false


******
**Actions**
******
- **GrabImages**
  Action to grab one or several images with desired image-intensity-setting each.
