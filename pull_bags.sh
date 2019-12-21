JETSON_ADDR=10.70.54.8

source ~/.bashrc
scp ubuntu@$JETSON_ADDR:/mnt/900_2/_2019-* bagfiles/
./safe_search.sh bagfiles/_2019-*

