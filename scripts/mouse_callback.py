clicked_coords = None

click_callback(event, x, y, flags, param):
    global clicked_coords
    if event == cv2.EVENT_RBUTTONUP:
        clicked_coords = (x, y)

cv2.namedWindow('preview')
cv2.setMouseCallback('preview', click_callback)
cv2.imshow('preview', img)

while clicked_coords is None:
    time.sleep(0.001)


