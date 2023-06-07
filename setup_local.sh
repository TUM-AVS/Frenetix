
IMAGE_NAME="frenetperformanceplanner"

if [ -z "$(docker images -q $IMAGE_NAME)" ]; then
  echo "Image does not exist. Building..."
  docker build -t $IMAGE_NAME .
else
  echo "Image exists. Skipping build..."
fi

docker run -td --name my-container $IMAGE_NAME
docker exec my-container /app/setup_script.sh
docker cp my-container:/app/dist/ .
docker stop my-container
docker rm my-container