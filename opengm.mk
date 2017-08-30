CWD:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TPD:=$(CWD)external
VER:=2.3.6
SD:=$(TPD)/opengm-$(VER)
IP:=$(TPD)/opengm-$(VER)-install


$(TPD)/opengm/include/opengm/opengm.hxx: $(IP)/include/opengm/opengm.hxx
	ln -fsT $(notdir $(IP)) $(TPD)/opengm

$(IP)/include/opengm/opengm.hxx: $(SD)/build/Makefile
	cd "$(<D)" && $(MAKE) install
	[ -f $@ ] && touch $@

$(SD)/build/Makefile: $(SD)/CMakeLists.txt
	mkdir -p "$(@D)"
	cd "$(@D)" && cmake .. -DCMAKE_INSTALL_PREFIX=$(IP)

$(TPD)/opengm-$(VER)/CMakeLists.txt:
	mkdir -p $(TPD)
	git clone https://github.com/opengm/opengm.git "$(@D)"
	cd "$(@D)" && git checkout v$(VER)
	touch $@

echo-%:
	@echo $($*)
