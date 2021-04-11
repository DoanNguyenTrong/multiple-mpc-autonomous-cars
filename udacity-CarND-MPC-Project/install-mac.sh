#! /bin/bash


# Update some configure for
# - MacOS Catalina
# - openssl@1.1.1k


brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PATH="/usr/local/opt/openssl@1.1/bin:$PATH"
export LDFLAGS="-L/usr/local/opt/openssl@1.1/lib"
export CPPFLAGS="-I/usr/local/opt/openssl@1.1/include"
export PKG_CONFIG_PATH="/usr/local/opt/openssl@1.1/lib/pkgconfig"
cd build
OPENSSL_VERSION=`openssl version -v | cut -d' ' -f2`
sudo ln -s /usr/local/Cellar/openssl@1.1/1.1.1k/lib/libcrypto.1.1.dylib /usr/local/lib/libcrypto.dylib
sudo ln -s /usr/local/Cellar/openssl@1.1/1.1.1k/lib/libssl.1.1.dylib /usr/local/lib/libssl.dylib
cmake -DOPENSSL_CRYPTO_LIBRARY="/usr/local/lib/libcrypto.dylib" -DOPENSSL_SSL_LIBRARY=/usr/local/lib/libssl.dylib -DOPENSSL_INCLUDE_DIR=/usr/local/opt/openssl@1.1/include -DOPENSSL_ROOT_DIR=$(brew --cellar openssl)/$OPENSSL_VERSION -DOPENSSL_LIBRARIES=$(brew --cellar openssl)/$OPENSSL_VERSION/lib ..
make -j $(sysctl -n hw.logicalcpu)
sudo make install
cd ../..
sudo rm -r uWebSockets
