.PHONY: build \
	build_judge \
	build_ros_packages \
	launch \
	prepare \
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

prepare:
	curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
	cargo install --git https://github.com/jerry73204/cargo-ament-build.git
	pip3 install git+https://github.com/jerry73204/colcon-ros-cargo.git@merge-colcon-cargo

clean:
	rm -rf build install log
