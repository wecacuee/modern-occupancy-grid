CWD:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TPD:=$(CWD)external
VER:=2.4.0
SD:=$(TPD)/gtsam-$(VER)
IP:=$(TPD)/gtsam-$(VER)-install

$(SD)/install/lib/libgtsam.so: $(SD)/build/Makefile
	cd "$(<D)" && $(MAKE) install

$(SD)/build/Makefile: $(SD)/CMakeLists.txt
	mkdir -p "$(@D)"
	cd "$(@D)" && cmake .. -DCMAKE_INSTALL_PREFIX=$(IP)

$(TPD)/gtsam-$(VER)/CMakeLists.txt:
	mkdir -p $(TPD)
	git clone https://bitbucket.org/gtborg/gtsam.git "$(@D)"
	cd "$(@D)" && git checkout $(VER)
	touch $@

echo-%:
	@echo $($*)
