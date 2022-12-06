//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Header fileok

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Int32.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <cmath>
#include <stdio.h>  
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include "acc_pkg/display_info.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Globális változók

vector<float> closest_interesting_object_x_range_container{1, 1, 1, 1, 1};
vector<float> wheel_rot_rad_prev{0, 0};
vector<float> wheel_spin_prev{0, 0};
vector<float> turning_radius_container(5);
float veh_vel_measured;
float veh_vel_control_lin;
float veh_vel_control_ang;
float avg_turning_radius;
float closest_object_range = 1;
float object_turning_radius;
float linear_vel;
float angular_vel;
float prev_closest_object_range;
float avg_rel_vel_of_closest_interesting_object;
vector<float> rel_velocityes(4);
int key_value;
bool acc_on_off = false;
bool lock_on = false;
bool object_detected = false;
bool turning = false;
bool prev_turning = false;
bool right_turn = false;
bool flop = false;
bool in_range = false;

geometry_msgs::Twist control_msg;
acc_pkg::display_info display_info;

ros::Publisher cmd_vel_pub;
ros::Publisher pygame_visu_pub;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Billentyűzet lenyomást érzékelő függvény
//  Amennyiben egy billentyű lenyomásra kerül 1-es értékkel tér vissza

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Beolvassa és vizsgálja a lenyomott billentyű ASCII értékét, és adott érték alapján állítja adott változó értékét
//  W: sebesség növelése, S: sebesség csökkentése, A: balra fordulás, D: jobbra fordulás, X: sebesség és fordulás értékek nullázása
//  1: ACC aktiválása, 2: sebességtartás aktiválása

void keypress(){
        
    if(kbhit()){

        key_value = getchar();                              // Lenyomott billentyű ASCII kódjának eltárolása key_value változóban, amennyiben kbhit() 1-es értékkel tér vissza

    }

    {   // Irányítás

        if(key_value == 119){                               // "W" lenyomására növeli linear_vel változó értékét, linear_vel maximum értéke 0.22 lehet 

            linear_vel = linear_vel + 0.01;

            if(linear_vel > 0.22){

                linear_vel = 0.22;

            }

        }

        if(key_value == 115){                               // "S" lenyomására csökkenti linear_vel változó értékét, linear_vel minimum értéke -0.22 lehet 

            linear_vel = linear_vel - 0.01;

            if(linear_vel < -0.22){

                linear_vel = -0.22;

            }

        }

        if(key_value == 97){                                // "A" lenyomására növeli angular_vel változó értékét, angular_vel maximális értéke 2.8 lehet 

            angular_vel = angular_vel + 0.1;

            if(angular_vel > 2.8){

                angular_vel = 2.8;

            }

        }

        if(key_value == 100){                               // "D" lenyomására csökkenti angular_vel változó értékét, angular_vel minimum értéke -2.8 lehet 

            angular_vel = angular_vel - 0.1;

            if(angular_vel < -2.8){

                angular_vel = -2.8;

            }

        }

        if(key_value == 120){                               // "X" lenyomására nullázza linear_vel és angular_vel változók értékét

            linear_vel = 0;
            angular_vel = 0;

        }

        if(linear_vel < 0.005 & linear_vel > -0.005){       // nullává teszi linear_vel változó értékét, amennyiben adott tartományon belüli értéket vesz vel
                                                            // float változó kerekítése miatt van rá szükség
            linear_vel = 0;

        }

        if(angular_vel < 0.005 & angular_vel > -0.005){     // nullává teszi angular_vel változó értékét, amennyiben adott tartományon belüli értéket vesz vel
                                                            // float változó kerekítése miatt van rá szükség
            angular_vel = 0;

        }

    }


    {    // ACC irányítás

        if(key_value == 49){                                // "1" lenyomására negálja acc_on_off bool változó értékét

            acc_on_off = !acc_on_off;

        }

        if(key_value == 50 & acc_on_off == true){          // Amennyiben acc_on_off változó igaz, "2" lenyomására negálja lock_on bool változó értékét

            lock_on = !lock_on;

        }


        if(key_value == 115 | key_value == 120){            // "S" vagy "X" lenyomására hamissá teszik lock_on és object_detected változó értéketi, ezzel deaktiválva sebességkövetés funkciót

            lock_on = false;
            object_detected = false;

        }

        if(acc_on_off == false){                            // Amennyiben acc_on_of értéke hamis, lock_on értékét hamisra állítja

            lock_on = false;
            veh_vel_control_lin = 0;

        }

    }

    key_value = 0;                                          // lenyomott billenytű értékének eltávolítása key_value változóból

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A turtlebot által mért aktuális x tengely irányú sebességet beletölti veh_vel_measured változóba
// Abban az esetben hívódik meg, amennyiben az "/odom" topic-ra publishálás történik
// A megadott típusú üzenetet a "data" lokális konstans tárolja el

void callback_odom(const nav_msgs::Odometry::ConstPtr& msg2){
    veh_vel_measured = round((msg2->twist.twist.linear.x) * 100) / 100;

    if(veh_vel_measured < 0.005 & veh_vel_measured > -0.005){

        veh_vel_measured = 0;

    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A turtlebot által mért kerékfordulatok eltérését vizsgálja és azok alapján megállapítja, hogy a robot egyenes vonalon, vagy íven halad
// Abban az esetben hívódik meg, amennyiben a "/joint_states" topic-ra publishálás történik (30Hz)
// A megedott típusú üzenetet a "data" lokális konstans tárolja el

void callback_jointstates(const sensor_msgs::JointState::ConstPtr& data){

    keypress();                                                                             // gomblenyomás vizsgálata
                                                                                            // tudható, hogy a /joint_states topicra a turtlebot 30Hz-n publishál, ezért
                                                                                            // érdemes itt meghívni a keypress()-t a billenytűlenyomás gyakori vizsgálata érdekében
    // lokális változók definiálása:
    vector<float> wheel_rot_rad_act(2);                                                     // indulás óta megtett kerék szögelfordulás értékek tárolása Rad-ban
    vector<float> wheel_spin_act(2);                                                        // számított keréksebességek tárolása m/s-ban

    // 2 tizedesjegyre kerekített Radián értékek betöltése a vektorba
    wheel_rot_rad_act[0] = data->position[0];                          //jobb oldali kerék szögelfordulása
    wheel_rot_rad_act[1] = data->position[1];                          // bal oldali kerék szögelfordulása

    // keréksebességek számítása két mérés közt eltelt időből, egymás utáni két mért Rad érték különbségéből és kerékátmérőből
    wheel_spin_act[0] = (wheel_rot_rad_act[0] - wheel_rot_rad_prev[0]) * 30 * 0.033;              // jobb oldali kerék sebessége
    wheel_spin_act[1] = (wheel_rot_rad_act[1] - wheel_rot_rad_prev[1]) * 30 * 0.033;              // bal oldali kerék sebessége


    // fordulás vagy egyenes haladás vizsgálata, amennyiben a két kerék számított sebessége egymástól eltér, akkor a robot fordul
    // a környezeti zavarások miatt, csak akkor mondjuk azt, hogy a robot kanyarodik, ha az aktuális és az előző keréksebességek is ugyan abba az irányban térnek el egymástól
    if(((wheel_spin_act[0] - wheel_spin_act[1]) > 0.008 & (wheel_spin_prev[0] - wheel_spin_prev[1]) > 0.008) | ((wheel_spin_act[0] - wheel_spin_act[1]) < -0.008 & (wheel_spin_prev[0] - wheel_spin_prev[1]) < -0.008)){

        turning = true;

        // ha a jobb oldali kerék forgása lassabb, akkor jobbra, ha a bal oldali kerék forgása lassabb, akkor balra fordul a robot
        if(wheel_spin_act[0] < wheel_spin_act[1]){

            right_turn = true;

        }
        else{

            right_turn = false;

        }

    }
    else{

        turning = false;

    }

    // fordulókör sugarának kiszámítása jobbra fordulás esetén
    if(turning == true & right_turn == true ){

        // az fordulósugarak tárolására használt vektor minden elemét eggyel hátrébb toljuk, a vektorban az előző 4 mérés fordulósugara és az aktuálisan számított fordulósugár kerül eltárolásra
        for(int i = 0; i < turning_radius_container.size()-1; i++){

            turning_radius_container[turning_radius_container.size()-1-i] = turning_radius_container[turning_radius_container.size()-2-i];

        }

        // aktuális fordulósugár kiszámítása a két kerék sebességkülönbségéből és az egymáshoz képesti távolságukból (a képlet miatt kell ketté bontani a jobbra és balra fordulás esetét a sugár számításakor)
        // az aktuálisan számított fordulósugár a fordulósugarak tárolására használt vektor 0. helyén kerül eltárolásra
        turning_radius_container[0] = 0.16 / ((wheel_spin_act[1] - wheel_spin_act[0]) / wheel_spin_act[1]);

        // átlagos fordulási sugár számítása 5 mérés alapján
        avg_turning_radius = round((turning_radius_container[0] + turning_radius_container[1] + turning_radius_container[2] + turning_radius_container[3] + turning_radius_container[4]) / 5 * 1000) / 1000;

    }

    // fordulókör sugarának kiszámítása balra fordulás esetén, ugyan az mint jobbra fordulás esetén, csak fordulósugár számításának képlete változott
    if(turning == true & right_turn == false){

        for(int i = 0; i < turning_radius_container.size()-1; i++){

            turning_radius_container[turning_radius_container.size()-1-i] = turning_radius_container[turning_radius_container.size()-2-i];

        }

        turning_radius_container[0] = (0.16 / ((wheel_spin_act[0] - wheel_spin_act[1]) / wheel_spin_act[0])) * (-1); 

        avg_turning_radius = ((turning_radius_container[0] + turning_radius_container[1] + turning_radius_container[2] + turning_radius_container[3] + turning_radius_container[4]) / 5) * (-1);

    }

    // aktuális kerékszögelfordulás és keréksebesség értékek áttöltése globális változókba, hogy következő lefutáskor megmaradjanak az akkor már előző mérésnek tekinthető mért adatok
    wheel_rot_rad_prev[0] = wheel_rot_rad_act[0];
    wheel_rot_rad_prev[1] = wheel_rot_rad_act[1];

    wheel_spin_prev[0] = wheel_spin_act[0];
    wheel_spin_prev[1] = wheel_spin_act[1];

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A turtlebot által mért lidar pontfelhőt feldolgozza, detektálja az objektumokat és számolja azok távolságát, sebességét és elvégzi a robot vezérlését
// Abban az esetben hívódik meg, amennyiben a "/scan" topic-ra publishálás történik (5Hz)


void callback_lidar(const sensor_msgs::LaserScan::ConstPtr& data){

// konzol törlése, kivezérelt sebességértékek és ACC állapotának kiíra console-ra
    system("clear");
    cout << "linear vel: " << linear_vel << "   angular_vel: " << angular_vel << '\n';
    cout << "ACC: " << acc_on_off << "  követés: " << lock_on << '\n' << '\n' << '\n';

    // lokális változók definiálása
    vector<float> actual_ranges;                                // lidar által detektált pontok távolsága
    vector<float> x_y(2);                                          // lidar által detektált pontok távolsága 2 dimenziós Descartes koordináta rendszerben
    vector<vector<float>> xy_ranges;                            // x_y értékek tárolása
    vector<int> object;                                         // egy objektumhoz tartozó pontok helyiértékei az "actual_ranges" vektorban
    vector<vector<int>> current_object_container;               // aktuális mérés alapján detektált objektumok pontértékeit tartalmazó vektorok tárolása
    vector<float> middle_point_coordinates(2);                  // adott detektált objektum középpontjának távolsága 2 dimenziós Descartes koordináta rendszerben
    vector<vector<float>> object_middle_point_container;        // detektált objektumok középpontjainak tárolása
    vector<int> temporary_vec;                                  // feldolgozáshoz létrehozott ideiglenes vector
    float x_range;                                              // adott pont X tengely irányú távolsága Descartes koordináta rendszerben (x_y 0. eleme)
    float y_range;                                              // adott pont Y tengely irányú távolsága Descartes koordináta rendszerben (x_Y 1. eleme)
    float x_distances = 0;                                      // adott objektum középpontjának X tengely irányú távolsága Descartes koordináta rendszerben (middle_point_coordinates 0. eleme)
    float y_distances = 0;                                      // adott objektum középpontjának Y tengely irányú távolsága Descartes koordináta rendszerben (middle_point_coordinates 1. eleme)
    float point_to_point_distance;                              // két egymás meletti helyiértékű pont egymástól való távolsága
    int point_num = 0;                                          // segéd változó
    int point_cnt = 0;                                          // segéd változó
    int lower_limiter;                                          // segéd változó
    int upper_limiter;                                          // segéd változó
    bool zero_ang_det_good = false;                             // segéd változó

    
                                                /////////// LIDAR PONTFELHŐ FELDOLGOZÁSA //////////

    // actual_ranges változó feltöltése a detektált pontok távolságával, amennyiben a pont messzebb van, mint 1m, az elmentett távolságérték 0 lesz (szűrés)
   for(int i = 0; i<data->ranges.size(); i++){

        actual_ranges.push_back(data->ranges[i]);

        if(actual_ranges[i] > 1){

            actual_ranges[i] = 0;

        }

    }

    // detektált pontok távolságának kiszámítása Descartes koorináta rendszerben mind a négy körnegyedben 
    // actual_ranges helyiértékeinek száma megegyezik az adott pont által bezárt szöggel
    for(int i = 0; i < actual_ranges.size(); i++){

        if(i < 90){

            x_range = actual_ranges[i] * cos(i*M_PI/180);
            y_range = (actual_ranges[i] * sin(i*M_PI/180));

        }

        if((i > 89) & (i < 180)){

            x_range = (actual_ranges[i] * sin((i-90)*M_PI/180)) * (-1);
            y_range = (actual_ranges[i] * cos((i-90)*M_PI/180));
            
        }

        if((i > 179) & (i < 270)){

            x_range = (actual_ranges[i] * cos((i-180)*M_PI/180)) * (-1);
            y_range = actual_ranges[i] * sin((i-180)*M_PI/180) * (-1);
            
        }

        if((i > 269) & (i < 360)){

            x_range = actual_ranges[i] * sin((i-270)*M_PI/180);
            y_range = actual_ranges[i] * cos((i-270)*M_PI/180) * (-1);
            
        }
      
        //  x_y vektor feltöltése adott pont távolságértékeivel, majd x_y beletöltése xy_ranges vektorba
        x_y[0] = x_range;
        x_y[1] = y_range;
        xy_ranges.push_back(x_y);

    }

    // egy objektumhoz tartozó pontok összegyűjtése és tárolása
    for(int i = 0; i< actual_ranges.size(); i++){

        // amennyiben egy objektum pontosan a robot előtt van, akkor a 0. szögértéken és a 359. szögértéken is lesz távolság detekciós
        // ezeket a távolságokat is össze kell haszonlítani és amennyiben úgy ítéljük meg, hogy egy objektumhoz tartoznak, akkor össze kell fűzni
        // a vektor elején és végén talált közeli pontokat egymással, ebben segítene a lower_limiter és upper_limiter változók
        if(i == 0){

            lower_limiter = 360;

        }
        else{

            lower_limiter = i;

        }
        if(i == 359){

            upper_limiter = -1;

        }
        else{

            upper_limiter = i;

        }

        // ha a vizsgált szögértéken lévő távolság nem 0, vagy a vizsgált szögértéknél egyel kisebb és egyel nagyobb távolság nem 0 (fals mérés szűrése), akkor 
        // az adott pont helyiértéke (szögértéke) belekerül az object vektorba és point_cnt számolja, hogy hány pont került bele az objet vektorba
        // ha nem teljesül a feltétel, akkor az object vektort beletölti a current_object_container vektor-ba és nulláza object vektort
        if((actual_ranges[i] != 0) | (actual_ranges[lower_limiter-1] != 0 & actual_ranges[upper_limiter+1] != 0)){

            object.push_back(i);
            point_cnt++;

        }
        else{

            if(actual_ranges[i-1] != 0){                          

                current_object_container.push_back(object);

                for(int j = 0; j < point_cnt; j++){

                    object.erase(object.begin());

                }

            point_cnt = 0;

            }
        }
    }

    // az objektumok detektálása után megvizsgálja, hogy a 0. és a 359. elemen is van-e érték
    // ha van rajta érték akkor biztos, hogy a current_object_containar elején és végén lévő két vektört egybe kell fűzni
    // hogy helyes sorrendben kövessék egymást a detektált pontok, az current_object_container[0] vektort kell az utolsó helyiértéken lévő vektor után fűzni
    if(actual_ranges[0] != 0 & actual_ranges[359] != 0){

        vector<int> temporary_vec = current_object_container[0];

        for(int j = 0; j<temporary_vec.size();j++){

            object.push_back(temporary_vec[j]);                     // ebben az esetben az object vektor még tartalmazza a ranges vektor utolsó helyiértékeit, így egyszerűen bővíthető 
                                                                    // current_object_container[0] vektor elemeivel
        }

        current_object_container.push_back(object);
        current_object_container.erase(current_object_container.begin());

    }

    // az elmentett objektumok vizsgálata
    // egy detektált objektumnak legalább 4 pontból kell álnia, ha egy objektum kevesebb pontot tartalmaz, akkor törlésre kerül (zajok kiszűrése)
    for(int i = 0; i < current_object_container.size(); i++){

        temporary_vec = current_object_container[i];
        
        if(temporary_vec.size() < 4){

            current_object_container.erase(current_object_container.begin()+i);

        }
        
    }

    // minden objektumban a pontok távolságának vizsgálata, objektumonként egy "középpont" meghatározása az objektumon belül lévő távolságértékek átlagának számításával
    for(int i = 0; i < current_object_container.size(); i++){

        for(int j = 0; j < current_object_container[i].size(); j++){

            point_num = current_object_container[i][j];

            x_distances = x_distances + xy_ranges[point_num][0];
            y_distances = y_distances + xy_ranges[point_num][1];

        }

        x_distances = x_distances / current_object_container[i].size();
        y_distances = y_distances / current_object_container[i].size();

        middle_point_coordinates[0] = x_distances;
        middle_point_coordinates[1] = y_distances;

        // objektum számított középpontjának távolságának eltárolása (Descartes koordináta rendszerben)
        object_middle_point_container.push_back(middle_point_coordinates);

        x_distances = 0;
        y_distances = 0;

    }

                                                                    ////////// SZABÁLYOZÁS //////////

    // szabályozás egyenes haladás esetén
    if(turning == false & angular_vel == 0){

        // Irány, számított sebesség kiírása
        cout << "Előre haladás\n";
        cout << "Sebesség: " << veh_vel_measured << " m/s\n" << '\n';

        // a program mindig a legközelebbi objektumot vizsgálja, csak az 1m-nél közelebb objektumokat vizsgálja, ha nincs objektum 1m-en belül, akkor biztosan 
        // nem indulhat el a szabályzás 
        float closest_interesting_object_x = 1;

        // összes objektum középpont vizsgálata, ha az adott objektum középpont Y irányba +- 0.1m-en belül van és X tengely irányú távolsága kisebb, mint az előző legközelebbi
        // objektumé, akkor ez az adott objektum válik a legközelebbi objektummá és X tengely irányú távolságértéke eltárolódik closest_interesting_object_x változóban
        for(int i = 0; i< object_middle_point_container.size(); i++){

            for(int j = 0; j < current_object_container[i].size(); j++){

                int object_select = current_object_container[i][j];

                if(xy_ranges[object_select][1] < 0.1 & xy_ranges[object_select][1] > -0.1){

                    in_range = true;

                }

            }

            if(in_range == true & object_middle_point_container[i][0] < 1 & object_middle_point_container[i][0] > 0.01){

                if(closest_interesting_object_x > object_middle_point_container[i][0]){

                    closest_interesting_object_x = object_middle_point_container[i][0];

                }

            }

            in_range = false;

        }


        // a vizsgált objektum sebességének meghatározásához tárolásra kerül az aktuális és az előző 4 mérés során számított X tengely irányú távolság
        // a closest_interesting_object_x_range_container vektorban, melynek 0. eleme mindig az aktuális mérés során számított távolság
        for(int i = 0; i < closest_interesting_object_x_range_container.size()-1; i++){

            closest_interesting_object_x_range_container[closest_interesting_object_x_range_container.size()-1-i] = closest_interesting_object_x_range_container[closest_interesting_object_x_range_container.size()-2-i];

        }

        closest_interesting_object_x_range_container[0] = closest_interesting_object_x;

            // objektum relatív sebességének tárolása 4 mérés hosszáig

        // relatív sebességeg számítása, mivel mindig ismert az adott, illetve az előző 4 mérés során számolt X tengely irányú távolság, ezért 4 relatív sebességérték számolható
        for(int i = 0; i < closest_interesting_object_x_range_container.size()-1; i++){

            rel_velocityes[i] = (closest_interesting_object_x_range_container[i] - closest_interesting_object_x_range_container[i+1]) / 0.2;

        }

        // detektált objektum relatív sebességértékeinek átlagolása és 3 tizedesjegyre történő kerekítése (mérési hiba és zavarás csökkentése)
        avg_rel_vel_of_closest_interesting_object = round((rel_velocityes[0] + rel_velocityes[1] + rel_velocityes[2] + rel_velocityes[3]) / 4 * 1000) / 1000;
        display_info.avg_rel_vel_of_closest_interesting_object = avg_rel_vel_of_closest_interesting_object;

        //amnnyiben az objektum sebessége adott értéken belül van, akkor az objektum állónak tekinthető
        if(avg_rel_vel_of_closest_interesting_object < 0.005 & avg_rel_vel_of_closest_interesting_object > -0.005){

            avg_rel_vel_of_closest_interesting_object = 0;

        }

        // ha billentyűzet segítségével be lett kapcsolva az ACC, akkor annak függvényében, hogy detektált-e objektumot 1m-es távolságon belül, kiír a konzolra
        if(acc_on_off == true){

            if(closest_interesting_object_x_range_container[0] == 1 & closest_interesting_object_x_range_container[1] == 1 & closest_interesting_object_x_range_container[2] == 1 & closest_interesting_object_x_range_container[3] == 1 & closest_interesting_object_x_range_container[4] == 1){
            
                if(acc_on_off == true){

                    cout << "Objektum keresése\n";

                }
                // billentyűzettel beállított értékek érvényre juttatása
                object_detected = false;

            }
            else{
                
                cout << "Objektum detektálva \nObjektum relatív segessége: " << avg_rel_vel_of_closest_interesting_object << " m/s\n";
                cout << "Objektum távolsága: " << closest_interesting_object_x_range_container[0] << " m\n";
                // szabályzás érvényre juttatása
                object_detected = true;

            }

        }

        // szabályzott értékek beállítása, ha a feltételek engedik
        if(lock_on == true & object_detected == true){

            if(closest_interesting_object_x_range_container[0] < 0.5){

                float u = avg_rel_vel_of_closest_interesting_object * 0.7;

                if(acc_on_off == true & object_detected == true){
                    display_info.control_sign = u * 0.2;
                }
                else{

                    display_info.control_sign = 0;
                }
                
                veh_vel_control_lin = veh_vel_measured + (u * 0.2);

                if(veh_vel_control_lin < 0.01){

                    veh_vel_control_lin = 0;

                }

                // a szabályzott sebességet nem engedi magasabbra állítani, mint a billentyűkkel beállított sebesség
                if(veh_vel_control_lin > linear_vel){

                    veh_vel_control_lin = linear_vel;

                }

            }
            else{

                veh_vel_control_lin = linear_vel;

            }

            
            // float kerekítés miatt kell, mert nem mindig 0 értéket ír ki alapból
            if(veh_vel_control_lin < 0.005){

                veh_vel_control_lin = 0;

            }

            // a szabályzott szögelfordulás érték előre haladásnál megegyezik a billenytűkkel beállított szögelfordulással, azaz 0
            veh_vel_control_ang = angular_vel;

            cout << "Objektum követése, vezérelt érték: " << veh_vel_control_lin <<" m/s\n";

        }

    }
    // szabályozás fordulás esetén
    else{
        
        // annak vizsgálata, hogy van-e olyan detektált pont, amely számított X és Y tengely távolságának alapján egybeesik a robot fordulási sugarával
        for(int i = 0; i < actual_ranges.size(); i++){

            if(actual_ranges[i] != 0){

                if(i < 90){                                       

                    object_turning_radius = sin((i*M_PI/180)) * avg_turning_radius;

                }

                if(i > 269){

                    object_turning_radius = sin(((i-269)*M_PI/180)) * avg_turning_radius;

                }

                // ha adott pont beleesik a fordulási sugár +- 5 cm értékbe, akkor elmenit a pont távolságát closest_object_range változóba
                // ha talál egy közelebbi pontot, amely szintén beleesik a fordulósugárba, akkor felülira closest_object_range értékét
                if(object_turning_radius > (avg_turning_radius - 0.05) & object_turning_radius < (avg_turning_radius + 0.05)){

                    float object_range;

                    if(right_turn == true & (i < 90 | i > 330)){

                        object_range = actual_ranges[i];

                    }

                    if(right_turn == false & (i > 269 | i < 30)){

                        object_range = actual_ranges[i];

                    }

                    if(object_range < closest_object_range & object_range > 0.01){

                        closest_object_range = object_range;

                    }

                }

            }

        }
        
        if(right_turn == true){

            cout << "Jobbra fordul, fordulókör: " << avg_turning_radius << " m\n";
            cout << "Sebesség: " << veh_vel_measured << " m/s\n" << '\n';

        }
        else{

            cout << "Balra fordul, fordulókör: " << avg_turning_radius << "m\n";
            cout << "Sebesség: " << veh_vel_measured << " m/s\n" << '\n';

        }

        // ha érzékelt pontot, amely 1m-nél közelebb van és egybeesik a fordulási sugárral
        if(closest_object_range < 1){

            if(acc_on_off == true){             

                cout << "Objektum detektálva, objektum fordulási sugara: " << object_turning_radius << " m\n";
                cout << "Objektum távolsága: " << closest_object_range << " m\n";

            }

            // szabályzás érvényre juttatása
            object_detected = true;


            // szabályzó értékek érvényre beállítása
            if(object_detected == true & lock_on == true){

                // akkor kezd el szabályozni, ha a vizsgált pont közelebb van 0.45m-nél
                if(closest_object_range < 0.45){

                    // első szabályozási kör során tengely körüli elfordulás billentyűzettel beállított értékének átadása a szabályozónak
                    if(flop == false){

                        veh_vel_control_ang = angular_vel;
                        flop = true;

                    }

                    // ha távolodunk a ponttól, akkor növeljük a sebesség és elfordulás értékét
                    if((prev_closest_object_range-closest_object_range) > 0.05){ 

                        veh_vel_control_lin = veh_vel_measured + 0.01;
                        veh_vel_control_ang = veh_vel_control_ang + (((angular_vel / (linear_vel*10)))*0.1);        

                        // nem enged magasabb sebességértéket kivezérelni a megadott sebességnél
                        if(veh_vel_control_lin > linear_vel){

                            veh_vel_control_lin = linear_vel;
                            veh_vel_control_ang = angular_vel;

                        }

                    }

                    // ha közeledünk a ponthoz, akkor csökkentjük a sebesség és tengely körüli elfordulás értékét
                    if((prev_closest_object_range-closest_object_range) < 0.05){ 

                        veh_vel_control_lin = veh_vel_measured - 0.01;
                        veh_vel_control_ang = veh_vel_control_ang - (((angular_vel / (linear_vel*10)))*0.1);

                        if(angular_vel < 0 & veh_vel_control_ang > 0){

                            veh_vel_control_ang = 0;

                        }

                        if(angular_vel > 0 & veh_vel_control_ang < 0){

                            veh_vel_control_ang = 0;

                        }

                    }

                }
                // ha adott értéknélm esszebb vagyunk a ponttól, akkor a szabályzó átveszi a billentyűkkel beállított értékeket
                else{

                    veh_vel_control_lin = linear_vel;
                    veh_vel_control_ang = angular_vel;

                }
            
                // float miatt kell
                if(veh_vel_control_lin < 0.005 & veh_vel_control_lin > -0.005){

                    veh_vel_control_lin = 0;
                    veh_vel_control_ang = 0;

                }

                cout << "Objektum követése, vezérelt érték: Sebesség:" << veh_vel_control_lin << " m/s    Szögelfordulás: " << veh_vel_control_ang << '\n';

                // tolatás, másik irányba történő fordulás megakadályozása
                if(veh_vel_control_lin == 0 | veh_vel_control_ang == 0){

                    veh_vel_control_lin = 0;
                    veh_vel_control_ang = 0;

                }

            }
            
        }
        // nincs pont detekció, mely egybeesik a fordulási sugárral
        else{

            object_detected = false;
            flop = false;

            if(acc_on_off == true){

                cout << "Objektum keresése\n";

            }

        }

        prev_closest_object_range = closest_object_range;

        if(acc_on_off == true & object_detected == true){
            display_info.control_sign = veh_vel_control_lin - veh_vel_measured;
        }
        else{
            display_info.control_sign = 0;
        }
        
    }

    // ha történik beavatkozás, akkor a szabályozott értékek kerülnek publisálásra, ha nincs szabályzás, akkor a billentyűk segítségével
    // beállított értékek jutnak érvényre
    

    if(acc_on_off == true & lock_on == true & object_detected == true){

        control_msg.linear.x = veh_vel_control_lin;
        control_msg.angular.z = veh_vel_control_ang;

        cmd_vel_pub.publish(control_msg);

    }
    else{

        control_msg.linear.x = linear_vel;
        control_msg.angular.z = angular_vel;

        cmd_vel_pub.publish(control_msg);

    }

    // a vizualizációhoz szükséges adatok betöltése a display_info üzenetbe és publishálás
    {
        display_info.kanyarodas = turning;
        display_info.jobbra_kanyarodas = right_turn;
        display_info.fordulokor_sugar = avg_turning_radius;

        vector<float> x_container;
        vector<float> y_container;
        vector<float> middlepoint_x_container;
        vector<float> middlepoint_y_container;

        for(int i = 0; i < xy_ranges.size(); i++){

            x_container.push_back(xy_ranges[i][0]);
            y_container.push_back(xy_ranges[i][1]);

        }
        
        for(int i = 0; i<object_middle_point_container.size(); i++){

            middlepoint_x_container.push_back(object_middle_point_container[i][0]);
            middlepoint_y_container.push_back(object_middle_point_container[i][1]);

        }

        display_info.pontok_x = x_container;
        display_info.pontok_y = y_container;
        display_info.objektum_kozeppont_x = middlepoint_x_container;
        display_info.objektum_kozeppont_y = middlepoint_y_container;

        display_info.linear_vel = linear_vel;
        display_info.angular_vel = angular_vel;
        display_info.acc_on_off = acc_on_off;
        display_info.lock_on = lock_on;
        display_info.veh_vel_measured = veh_vel_measured;
        display_info.object_detected = object_detected;
        if(acc_on_off == true & object_detected == true){

            

            if(turning == false & angular_vel == 0){
                display_info.object_speed = veh_vel_measured + avg_rel_vel_of_closest_interesting_object;
                display_info.closest_object_range = closest_interesting_object_x_range_container[0];;
            }
            else{
                display_info.closest_object_range = closest_object_range;
                display_info.object_speed =  (veh_vel_measured + ((closest_object_range - prev_closest_object_range) / 0.2))/10;
            }

            prev_closest_object_range = closest_object_range;


            display_info.veh_vel_control_lin = veh_vel_control_lin;

        }
        else{

            display_info.closest_object_range = 0;
            display_info.object_speed = 0;
            display_info.veh_vel_control_lin = 0;

        }
        display_info.veh_vel_control_ang = veh_vel_control_ang;
        display_info.object_turning_radius = object_turning_radius;

        closest_object_range = 1;
    }

    pygame_visu_pub.publish(display_info);

}


int main(int argc, char **argv){

    ros::init(argc, argv, "acc_node");
    ros::NodeHandle n;

    ros::Subscriber sub_lidar = n.subscribe("/tb3_0/scan", 1, callback_lidar);                    // feliratkozik a "/scan" topicra és meghívja "callback_scan" függvényt
    ros::Subscriber sub_wheel = n.subscribe("/tb3_0/joint_states", 1, callback_jointstates);      // feliratkozik a "/joint_states" topicra és meghívja "callback_jointstates" függvényt
    ros::Subscriber sub_odom = n.subscribe("/tb3_0/odom", 1, callback_odom);                      // feliratkozik az "/odom" topicra és meghívja "callback_odom" függvényt

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1);                         // publishál "cmd_vel" topic-ra, ha meghívódik
    pygame_visu_pub = n.advertise<acc_pkg::display_info>("/pygame_visu", 1);                // publishál "pygyme_visu" topic-ra, ha meghívódik

    // ros::Subscriber sub_lidar = n.subscribe("/scan", 1, callback_lidar);                    // feliratkozik a "/scan" topicra és meghívja "callback_scan" függvényt
    // ros::Subscriber sub_wheel = n.subscribe("/joint_states", 1, callback_jointstates);      // feliratkozik a "/joint_states" topicra és meghívja "callback_jointstates" függvényt
    // ros::Subscriber sub_odom = n.subscribe("/odom", 1, callback_odom);                      // feliratkozik az "/odom" topicra és meghívja "callback_odom" függvényt

    // cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);                         // publishál "cmd_vel" topic-ra, ha meghívódik
    // pygame_visu_pub = n.advertise<acc_pkg::display_info>("/pygame_visu", 1);                // publishál "pygyme_visu" topic-ra, ha meghívódik

    ros::spin();
    return 0;
}
