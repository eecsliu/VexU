for method in ("bottom-to-top"):
		# sort the contours
		(cnts, boundingBoxes) = contours.sort_contours(cnts, method=method)
	clone = output.copy()

	# loop over the sorted contours and label them
	for (i, c) in enumerate(cnts):
		sortedImage = contours.label_contour(clone, c, i, color=(240, 0, 159))

	# show the sorted contour image
	cv2.imshow(method, sortedImage)		