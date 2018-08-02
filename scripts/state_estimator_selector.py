if __name__ == '__main__':
    print 'Select which state estimator you wish to use:\n'
    print '1. Motion Capture'
    print '2. Velocity and Position Estimator'
    print '3. Localization'
    print '4. Unscented Kalman Filter'

    selection = raw_input('Enter a number:\t')
    if selection == '1':
        execfile('mocap_state_estimator.py')
    elif selection == '2':
        execfile('flow_pub_transform.py')
    elif selection == '3':
        execfile('picam_localization_distance.py')
    elif selection == '4':
        pass
    else:
        print 'invalid input'
