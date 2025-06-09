# Building the projects:
# mkdir varmint_build      && cd varmint_build      && cmake .. -DBOARD_TO_BUILD=varmint      && make -j
# mkdir pixracer_pro_build && cd pixracer_pro_build && cmake .. -DBOARD_TO_BUILD=pixracer_pro && make -j
#
# alternate for make -j above is cmake --build . -j (note only one .)
# make -j OR cmake --build . -j
# to clean:
# make clean -j  OR  make --build . --target clean -j
#
# unit test:
# build:
# mkdir test_build         && cd test_build         && cmake .. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j
# test:
# ./test/unit_tests


sudo rm -r varmint_build
mkdir varmint_build      && cd varmint_build      && cmake .. -DBOARD_TO_BUILD=varmint      && make -j