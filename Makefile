IMAGE := espressif/idf:release-v5.4

PORT ?= /dev/ttyACM0

DOCKER := docker run --rm -v $(CURDIR):/project -w /project

.PHONY: build flash monitor clean

build:
	$(DOCKER) $(IMAGE) bash -c "git config --global --add safe.directory /project && idf.py build"

flash:
	$(DOCKER) --device $(PORT) $(IMAGE) idf.py -p $(PORT) flash

monitor:
	$(DOCKER) -it --device $(PORT) $(IMAGE) idf.py -p $(PORT) monitor

clean:
	$(DOCKER) $(IMAGE) bash -c "git config --global --add safe.directory /project && idf.py fullclean"
