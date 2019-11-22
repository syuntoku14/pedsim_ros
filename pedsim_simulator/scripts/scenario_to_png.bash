for infile in ~/pedsim_ws/src/pedsim_ros/pedsim_simulator/scenarios/*.xml ; 
do python scenario_to_png.py $infile ;
done
