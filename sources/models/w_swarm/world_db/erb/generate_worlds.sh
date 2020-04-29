rb=$1
for f in *.erb
do
of=$(sed 's/.erb//g' <<< $f)
erb rb=$rb $f > ../r$rb/$of
echo $of
done