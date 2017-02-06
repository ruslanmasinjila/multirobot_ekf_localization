Multirobot Localization Using Extendend Kalman Filter.

USAGE:      RunMe
            >Change number of robots, simulation length and number of runs 

CONCEPT:    A group of N robots with known but uncertain initial poses move randomly
            in an open, obstacle-free environment. As some of robots (one or more) move, 
            the rest of robots (at least one), remain stationary and act as landmarks to
            the moving robots, and vice versa.

            More information is available on Chapter 3 (page 21 to 58)
            https://www.ruor.uottawa.ca/bitstream/10393/35489/1/masinjila_ruslan_2016_thesis.pdf
            

            For each robot, the following assumptions and models were made:

            motion model: unicycle (2-wheeled robot).
            measurement model: relative distance (rho) and relative angle (phi).
            encoder noise: gaussian and linearly proportional to the distance moved by the wheels.
            range sensor noise: gaussian in rho and phi.


STATUS:     February 4th 2017 [version1]
            > general case for multirobot localization involving N robots where N>=2.

            February 5th 2017 [version2]
            > allows the simulation to be repeated numRuns times under the same initial 
                conditions, control inputs, and sequence of movements.
            > calculates and displays estimated results averaged over numRuns times.
            > computes and displays ANEES averaged over numRuns.

            February 6th 2017 [version3]
            > computes and display absolute errors in X and Y coordinates of all robots.

TODO:       > display actual and estimated 2D error ellipses around estimated robot poses.
            > add covariance inflation index for tuning the ekf algorithm.