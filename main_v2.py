'''
PF Converging
    Max: 40 Iterations
Go to pickup zone
    -Set up CMAP (new start and new goals and add obstacle) 
    -say ready for delivery    
Pick up cube
    -check to see if it is visible by cube
        -rotate robot if it isn't visible till it is
        -only then do you dock with the cube
Go to drop Off
    -Set up CMAP (new start and new goals and add obstacle)    
Repeat

Misc
    - make sure to do .result() for async functions
'''