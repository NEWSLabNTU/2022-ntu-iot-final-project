.PHONY: build \
	build_judge \
	build_ros_packages \
	launch \
	clean

build: build_ros_packages build_judge

build_ros_packages:
	colcon build \
		--symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release \
		--cargo-args --release
build_judge:
	cd judge && \
	poetry install

clean:
	rm -rf build install log
