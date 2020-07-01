# exposure statistics for a multi-exposure HDR sequence
ImageExposureStatistics[] exposures
uint32 underExposed 		# number of pixels underexposed in longest exposure
uint32 overExposed  		# number of pixels overexposed in shortest
float64 meanIrradiance 		# mean irradiance of all properly exposed pixels in all images in the sequence
float64 minMeasurableIrradiance
float64 maxMeasurableIrradiance
float64 minObservedIrradiance
float64 maxObservedIrradiance

