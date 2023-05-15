nvidia-docker run -itd \
       	-v /home/zhisheng/new_disk/Documents/carla:/workspace/carla_source \
	-v /home/zhisheng/new_disk/Documents/xlab_projects/PASS_RUNNER:/workspace/PASS_RUNNER \
	--name pylot-challenge \
	-p 20022:22 \
	--net host \
	erdosproject/pylot-carla-challenge \
	/bin/bash

nvidia-docker exec -it pylot-challenge /bin/bash

