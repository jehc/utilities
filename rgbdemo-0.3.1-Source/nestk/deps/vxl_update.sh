if test $# -ne 1; then
  echo "Usage: $0 new_vxl_path"
  exit 1
fi

for f in `svn list -R vxl | grep -v CVS`; do 
    cp "$1/$f" "vxl/$f"
done
