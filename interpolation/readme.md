# waypoint_interpolation
./waypoint_interpolation.py (-l|-c|-cs) \[-g] \[-d] \[-o] -p file\_path  
-l | -c | -cs : 모델 선택. l=linear, c = cubic, cs = cubic spline  
-g : 그래프 표시  
-d : 디버그용 옵션(콘솔 출력)  
-o : 결과물 파일 출력(file\_name.itp.txt 로 저장됨)  
-p : file_name 지정. 바로 뒤에 경로가 따라와야함  
  
./waypoint_interpolation -l -g -o -p paths/path1.txt  
  
# waypoint_mode
<<<<<<< HEAD
./waypoint_mode.py -m number -p file\_path
path의 모든 waypoint에 추가 번호 number를 부여 (x y -> x y number)

./waypoint_mode.py -m number -r a:b -p file\_path  
path의 a번째 waypoint부터 (b-1)번째 waypoint까지 number를 부여  
1 base indexing (10:15 -> 9, 10, 11, 12, 13 in numpy array)

=======
./waypoint_mode.py -m number -p file\_path  
path의 모든 waypoint에 추가 번호 number를 부여 (x y -> x y number)  
  
>>>>>>> aad5a10f15be28d2f86469cebd4fb4a2b660519f
# waypoint_reverse
./waypoint_reverse.py file\_path  
file\_path의 waypoint 리스트를 뒤집어서 변환 및 저장.  
file\_path.rvs.txt로 저장  
  
# waypoint_translation
./waypoint_translation -a num1 -b num2 -p file\_path  
file\_path의 모든 waypoint를 (-a,-b)만큼 이동  
 
# waypoint_distance_hist
./waypoint_distance_hist file_\path
file\_path의 waypoint 사이의 거리를 막대그래프로 보여줌
