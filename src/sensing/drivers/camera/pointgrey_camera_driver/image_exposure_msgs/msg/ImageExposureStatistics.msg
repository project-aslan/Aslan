# message for exposure statistics reported a single image
time stamp         # image time stamp
string time_reference # The name of the reference clock for this message's timestamp.
float32 shutterms  # shutter durations in ms
float32 gaindb     # gain in decibels
# pixel exposure and latency statistics
uint32 underExposed # number of pixels underexposed
uint32 overExposed  # number of pixels overexposed
statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image
statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames
# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))
float64 meanIrradiance
float64 minMeasurableIrradiance
float64 maxMeasurableIrradiance
float64 minObservedIrradiance
float64 maxObservedIrradiance


