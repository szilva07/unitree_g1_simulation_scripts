# Low level irányítás

Ugyanazt a trajectory fájlt a unitree mujoco és a mujoco menagerie megoldással is le lehet szimulálni.

A g1_clap_trajectory.py egy tapsolás szerű mozgás.

A g1_bend_trajectory.py egy olyan mozgás pozícióit adja meg, amitől a robot előre hajol, összezárja a kezeit, mintha megfogna valamit, majd kiegyenesedik.

Én a scripteket a /unitree_mujoco/example/python mappába raktam, ezért a parancsokban az van, hogy onnan indítsa el.

WSL2-ben Ubuntu 24.04-es verziót és python 3.12.3-as verziót használtam.

## Virtuális környezet

Virtuális környezet:
```
cd ~
sudo apt update

sudo apt install python3-venv

python3 -m venv unitree_env

source ~/unitree_env/bin/activate
```

cyclonedds:
```
cd ~
sudo apt install -y \
    cmake \
    gcc \
    g++ \
    make \
    python3-dev

git clone --branch releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git

cd ~/cyclonedds

mkdir build
cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install \
  -DBUILD_DDSC=ON

cmake --build . --target install -j$(nproc)

echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.bashrc

source ~/.bashrc
```

unitree_sdk2_python:
```
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

cd ~/unitree_sdk2_python
pip install -e .
```

## Unitree Mujoco

https://github.com/unitreerobotics/unitree_mujoco#mujoco

Mujoco elindítása:
```
cd ~/unitree_mujoco/simulate/build

./unitree_mujoco -r g1 -i 0 -n eth0
```

Másik terminálban:
```
cd ~/unitree_mujoco/example/python

source ~/unitree_env/bin/activate

python3 g1_trajectory_player.py
```
Ezután a szimulációban párszor a Reset gombot meg kell nyomni, amíg nem áll meg a robot.

Először alaphelyzetbe áll a robot, majd Enter nyomására elkezdi végighajtani a mozgást. Ha a végére ért, megint az Enter nyomására újrakezdi.

A g1_trajectory_player.py elvileg a valós roboton is működik, csak a kikommentezett motionswitcher részt vissza kell rakni.

## Mujoco Menagerie

https://github.com/google-deepmind/mujoco_menagerie

Csak a szükséges fájlok letöltése a repóból:
```
cd ~
git clone --filter=blob:none --no-checkout https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco_menagerie

git sparse-checkout init --cone
git sparse-checkout set unitree_g1

git checkout main

source ~/unitree_env/bin/activate

python3 -m pip install mujoco
```

Szimuláció elindítása:
```
cd ~/unitree_mujoco/example/python

source ~/unitree_env/bin/activate

python3 g1_menagerie_trajectory_player.py
```

Pár másodperc várakozás után elkezdi végrehajtani a mozgást, és addig ismétli, amíg nincs leállítva.