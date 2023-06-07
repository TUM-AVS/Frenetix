IMAGE_NAME="frenetperformanceplanner"
CONTAINER_NAME="my-container"

if [ -z "$(docker images -q $IMAGE_NAME)" ]; then
  echo "Image does not exist. Building..."
  docker build -t $IMAGE_NAME .
else
  echo "Image exists. Skipping build..."
fi

if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
  echo "Container already exists. Removing..."
  docker stop $CONTAINER_NAME
  docker rm $CONTAINER_NAME
fi

docker run -td --name $CONTAINER_NAME $IMAGE_NAME
docker exec $CONTAINER_NAME /app/setup_script.sh
docker cp $CONTAINER_NAME:/app/dist/ .
docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME
