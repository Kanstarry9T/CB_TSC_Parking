netgenerate --grid --grid.x-number=15 --grid.y-number=10 \
 --grid.y-length=500 --grid.x-length=600 --default.lanenumber=3 --turn-lanes=2 --turn-lanes.length=100 --grid.attach-length=500 --default-junction-type traffic_light_right_on_red --output-file=grid.net.xml

python ../randomTrips.py -n grid.net.xml -o trips_grid.xml -p 0.06 --validate