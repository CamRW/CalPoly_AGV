CONTAINER_NAME="rock_humble:latest"

docker run -it --rm \
	--privileged \
	--network host \
	-v /dev/*:/dev/* \
	-v /home/rock/workspaces:/workspaces \
	$CONTAINER_NAME
