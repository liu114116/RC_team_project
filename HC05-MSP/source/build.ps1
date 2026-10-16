param(
    [Parameter(Mandatory=$true)][string]$Jdk,
    [Parameter(Mandatory=$true)][string]$BuildTools,
    [Parameter(Mandatory=$true)][string]$AndroidJar
)
$ErrorActionPreference='Stop'
$root=$PSScriptRoot
$build=Join-Path $root 'build'
New-Item -ItemType Directory -Force -Path "$build/classes","$build/dex" | Out-Null
function Check {if($LASTEXITCODE -ne 0){throw "Build step failed: $LASTEXITCODE"}}
& "$Jdk/bin/javac.exe" -encoding UTF-8 -source 8 -target 8 -classpath $AndroidJar -d "$build/classes" "$root/src/cn/rcproject/msp/Msp.java" "$root/src/cn/rcproject/msp/MainActivity.java" "$root/tests/MspTest.java"
Check
& "$Jdk/bin/java.exe" -cp "$build/classes" MspTest
Check
& "$BuildTools/aapt.exe" package -f -M "$root/AndroidManifest.xml" -I $AndroidJar -S "$root/res" -A "$root/assets" -F "$build/unsigned.apk"
Check
& "$Jdk/bin/jar.exe" cf "$build/classes.jar" -C "$build/classes" cn
Check
& "$Jdk/bin/java.exe" -cp "$BuildTools/lib/d8.jar" com.android.tools.r8.D8 --lib $AndroidJar --min-api 26 --output "$build/dex" "$build/classes.jar"
Check
Push-Location "$build/dex"
try { & "$BuildTools/aapt.exe" add "$build/unsigned.apk" classes.dex; Check } finally {Pop-Location}
& "$BuildTools/zipalign.exe" -f 4 "$build/unsigned.apk" "$build/aligned.apk"
Check
if(!(Test-Path -LiteralPath "$build/debug.jks")){
 & "$Jdk/bin/keytool.exe" -genkeypair -keystore "$build/debug.jks" -storepass android -keypass android -alias androiddebugkey -dname 'CN=RC Project Debug' -keyalg RSA -keysize 2048 -validity 3650
 Check
}
& "$Jdk/bin/java.exe" -jar "$BuildTools/lib/apksigner.jar" sign --ks "$build/debug.jks" --ks-pass pass:android --out "$build/HC05-MSP-debug.apk" "$build/aligned.apk"
Check
& "$Jdk/bin/java.exe" -jar "$BuildTools/lib/apksigner.jar" verify --verbose "$build/HC05-MSP-debug.apk"
Check
