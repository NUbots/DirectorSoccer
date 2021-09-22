# Maintainer: Jingbei Li <i@jingbei.lli>
pkgname='kaldi'
pkgdesc='Speech recognition research toolkit'
# The last version part is the short git ref
pkgver=5.5.r9195.3b8a97d85
pkgrel=1
depends=('cblas' 'kaldi-openfst' 'lapack' 'python2')
optdepends=('cuda' 'kaldi-irstlm' 'kaldi-kaldi_lm' 'kaldi-sctk' 'kaldi-sph2pipe' 'kaldi-srilm')
makedepends=('git' 'wget' 'sed')
arch=('x86_64' 'i686')
url='https://github.com/kaldi-asr/kaldi'
license=('APACHE')
source=("git+${url}")
sha256sums=('SKIP')

# pkgver () {
#     cd "${pkgname}"
#     (
#         set -o pipefail
#         echo -n `cat src/.version`.
#         git describe --long 2>/dev/null | sed 's/\([^-]*-g\)/r\1/;s/-/./g' ||
#             printf "r%s.%s" "$(git rev-list --count HEAD)" "$(git rev-parse --short HEAD)"
#     )
# }

prepare(){
    cd $srcdir/$pkgname
    # Use this specific version instead of build from whatever is in the master branch.
    git checkout 3b8a97d859464912ddc50877f7280048f654c275

    find . -name '*.py' -exec sed '1s/python/python2/' -i {} \;

    if (false && pacman -Q cuda &> /dev/null); then
        msg2 "Compiling with CUDA support"
        _cuda_config_opts="--cudatk-dir=/opt/cuda"
    else
        msg2 "Compiling without CUDA support"
        _cuda_config_opts="--use-cuda=no"
    fi

    # Removing static libs in https://github.com/kaldi-asr/kaldi/blob/8f94bd0698d503c5c18e02f8fd4209b6802ff17a/src/makefiles/linux_clapack.mk#L16
    sed '/^LDLIBS = /s/\$(CLAPACKLIBS)//' -i src/makefiles/linux_clapack.mk
}

build () {
    cd $srcdir/$pkgname/src
    #CXX=/opt/cuda/bin/g++ \
    LDFLAGS='-lcblas -llapack' \
    ./configure $_cuda_config_opts \
        --shared \
        --fst-root=/opt/kaldi/tools/openfst \
        --clapack-root=/usr
    #   --cub-root=/usr/include \
    make depend
    make
}

package () {
    cd  $srcdir/$pkgname

    for i in src/lib/*.so
    do
        mv `realpath $i` $i
    done
    rm -f src/*/*.{cc,cu,o,a,orig}
    rm -r src/{doc,feat/test_data,lm/examples,lm/test_data,makefiles,onlinebin,probe}
    find src \( \
        -name 'Makefile*' \
        -or -name 'README' \
        -or -name 'CMake*' \
        -or -name '*.mk' \
        -not -name 'kaldi.mk'\
        \) -exec rm {} \;
    find src -maxdepth 1 -type f -not -name 'kaldi.mk' -exec rm {} \;
    rm -r tools/{ATLAS_headers,CLAPACK,INSTALL,Makefile}

    sed "s|$srcdir|/opt|g" -i `grep $srcdir . -rIl`
    find . -name 'path.sh' -exec sed 's?^\(export KALDI_ROOT\)=.*$?\1=/opt/'$pkgname'?' -i {} \;
    echo "export OPENFST=$(find /opt/$pkgname/tools -type d -name 'openfst*')" >> tools/env.sh
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:${OPENFST}/lib' >> tools/env.sh
    echo "export IRSTLM=/opt/$pkgname/tools/irstlm" >> tools/env.sh
    echo 'export PATH=${PATH}:${IRSTLM}/bin' >> tools/env.sh

    install -dm755 "$pkgdir"/etc/ld.so.conf.d/
    echo "/opt/$pkgname/src/lib" > "$pkgdir"/etc/ld.so.conf.d/$pkgname.conf

    mkdir -p $pkgdir/opt/$pkgname
    cp -r src egs tools $pkgdir/opt/$pkgname
}
