#!/bin/bash
# Build OBD2 Scanner APK using aapt2 + d8 + apksigner
set -e

PROJECT=~/obd2-scanner
SRC=$PROJECT/app/src/main
ANDROID_JAR=$HOME/android-sdk/platforms/android-28/android.jar
BUILD=$PROJECT/build
GEN=$BUILD/gen
OBJ=$BUILD/obj

echo "=== MobileCLI OBD2 Scanner Build ==="

# Clean
rm -rf $BUILD
mkdir -p $GEN $OBJ $BUILD/dex

# Step 1: Compile resources with aapt2
echo "[1/6] Compiling resources..."
aapt2 compile --dir $SRC/res -o $BUILD/res.zip

# Step 2: Link resources and generate R.java
echo "[2/6] Linking resources..."
aapt2 link -o $BUILD/obd2-base.apk \
    -I $ANDROID_JAR \
    --manifest $SRC/AndroidManifest.xml \
    --java $GEN \
    $BUILD/res.zip

# Step 3: Compile Java
echo "[3/6] Compiling Java..."
javac --release 8 \
    -classpath $ANDROID_JAR \
    -d $OBJ \
    $GEN/com/mobilecli/obd2/R.java \
    $SRC/java/com/mobilecli/obd2/OBD2Scanner.java

# Step 4: Convert to DEX
echo "[4/6] Converting to DEX..."
d8 --min-api 21 --lib $ANDROID_JAR --output $BUILD/dex/ $OBJ/com/mobilecli/obd2/*.class

# Step 5: Add DEX to APK
echo "[5/6] Packaging APK..."
cp $BUILD/obd2-base.apk $BUILD/obd2-unsigned.apk
cd $BUILD/dex
zip -j $BUILD/obd2-unsigned.apk classes.dex
cd $PROJECT

# Step 6: Align and sign
echo "[6/6] Signing APK..."
zipalign -f 4 $BUILD/obd2-unsigned.apk $BUILD/obd2-aligned.apk

if [ ! -f $PROJECT/debug.keystore ]; then
    keytool -genkeypair -v \
        -keystore $PROJECT/debug.keystore \
        -storepass android \
        -keypass android \
        -keyalg RSA -keysize 2048 \
        -validity 10000 \
        -alias obd2 \
        -dname "CN=MobileCLI,OU=OBD2,O=MobileCLI,C=US"
fi

apksigner sign \
    --ks $PROJECT/debug.keystore \
    --ks-pass pass:android \
    --key-pass pass:android \
    --ks-key-alias obd2 \
    --out $BUILD/OBD2Scanner.apk \
    $BUILD/obd2-aligned.apk

cp $BUILD/OBD2Scanner.apk /sdcard/Download/OBD2Scanner.apk

echo ""
echo "=== BUILD SUCCESS ==="
echo "APK: $BUILD/OBD2Scanner.apk"
echo "Copied to: /sdcard/Download/OBD2Scanner.apk"
echo ""
echo "Install: pm install $BUILD/OBD2Scanner.apk"
