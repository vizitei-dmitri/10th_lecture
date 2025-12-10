# 10th_lecture — UR3 + Harmonic Drive (MJCF)




### 1.2. Виртуальное окружение 

Создать окружение в корне репозитория если текущее не рабоатет (но по идее должно работать и моё):

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip

```

Активировать:

```bash
# Linux
source .venv/bin/activate


Установить зависимости (минимальный набор + mojuco):

```bash
pip install mujoco
pip install numpy lxml mujoco
```

---

## 2. Структура репозитория

```text
10th_lecture/
├── .venv/                   # виртуальное окружение Python
├── README.md                # этот файл
└── ur_description/
    ├── meshes/              # STL/DAE сетки звеньев UR3
    ├── meta-information.json
    ├── universalRobots.rights
    └── urdf/
        ├── generate_models.py
        ├── harmonic_drives.py
        ├── mjmodel.xml #оригинальная xml
        ├── universalUR3.urdf
        ├── run_ur10_mujoco.py
        ├── ur3_hd_replace_motor_single.xml   # основная модель с HD
        ├── generated_models/                 # сгенерированные варианты UR3
        └── __pycache__/
```

Кратко по важным файлам:
### ipynb файл:
 - **'report'**
 тут изложил очень подробно процесс подбора мотора, советую глянуть!
### `ur_description/urdf/`

- **`universalUR3.urdf`, `universalUR5.urdf`, `universalUR10.urdf`**  
  Оригинальные URDF-описания UR-манипуляторов от Universal Robots.  
  UR3 отсюда был экспортирован в MJCF (`mjmodel.xml`) через `mujoco.viewer` 
- **`mjmodel.xml`**  
  Базовая MJCF-модель UR3 (ещё без Harmonic Drive).  
  Это *шаблон*, поверх которого скрипт `generate_models.py` строит новые модели.

- **`harmonic_drives.py`**  
  Описание выбранных Harmonic Drive SHA-SG для суставов UR3:
  - dataclass `HarmonicDrive` с параметрами:
    - `name` — строковое имя модели,
    - `gear_ratio` — передаточное число (в нашей задаче 101:1),
    - `armature_inertia` — приведённая инерция GD²/4 (kg·m²) из каталога,
    - `max_torque` — максимальный выходной момент (N·m);
  - конкретные модели: `SHA20A_101`, `SHA25A_101`, `SHA32A_101`
  - словарь `joint_variants`:
    - какие SHA-актуаторы можно ставить на каждый сустав  
      (`shoulder_pan_joint`, `elbow_joint`, `wrist_1_joint`, ...)

- **`generate_models.py`**  
  Главный скрипт для модификации MJCF-модели UR3. Делает несколько шагов:

  1. Загружает `mjmodel.xml` и строит дерево XML (`lxml.etree`).
  2. Находит все `joint` и их родительские `body` (через `map_joint_to_body`).
  3. Для каждого сустава:
     - читает исходные `mass` и `diaginertia` звена;
     - через `OLD_MOTORS` (гипотетическая модель старого мотора как цилиндра)
       вычитает вклад старого мотора из инерции звена;
     - добавляет новый актуатор Harmonic Drive (также как цилиндр: масса, радиус, длина)  
       и пересчитывает `mass` и `diaginertia` звена;
     - записывает в `joint` поле `armature` = `hd.armature_inertia`  
       (это приведённая инерция GD²/4 из каталога);
     - создаёт `<actuator><general .../></actuator>` с:
       - `gear = hd.gear_ratio` (101.0),
       - `ctrlrange = [−T_motor_max, T_motor_max]`, где  
         `T_motor_max = max_torque / gear_ratio`,  
         так что **максимальный момент в суставе** равен `max_torque` из каталога.
  4. Сохраняет:
     - одну базовую модель `ur3_hd_replace_motor_single.xml` (один фиксированный набор SHA),
     - набор вариантов в `generated_models/` (все комбинации из `joint_variants`).

- **`run_ur10_mujoco.py`**  
  Пример Python-скрипта для запуска модели UR10 в MuJoCo.  
  По аналогии можно сделать такой же скрипт для UR3-моделей, которые генерируются выше.

- **`ur3_hd_replace_motor_single.xml`**  
  Основная MJCF-модель UR3 с SHA-SG:
  - старые моторы UR3 представлены как цилиндры и их вклад в инерцию вычтен;
  - новые Harmonic Drive добавлены как цилиндры с массой и габаритами из каталога;
  - поле `armature` и секция `<actuator>` настроены по данным из каталога SHA-SG.

- **`generated_models/`, `ur3_hd_variants/`**  
  Папки с набором сгенерированных вариантов UR3 (разные комбинации SHA20/25/32 на суставах).  
  Каждый XML — отдельная конфигурация актуаторов.

---

## 3. Как сгенерировать модели UR3 с Harmonic Drive

1. Активировать виртуальное окружение в корне репозитория:

   ```bash
   source .venv/bin/activate
   ```

2. Перейти в папку с MJCF/URDF:

   ```bash
   cd ur_description/urdf
   ```

3. Запустить генерацию:

   ```bash
   python generate_models.py
   ```

После этого:

- в текущей папке появится (или обновится) файл  
  **`ur3_hd_replace_motor_single.xml`** — базовая модель UR3 с выбранными SHA;
- в **`generated_models/`** появится несколько файлов вида:

  ```text
  ur3_sha_sg_101_variant_01.xml
  ur3_sha_sg_101_variant_02.xml
  ...
  ```

Каждый из них соответствует своей комбинации `SHA20A_101 / SHA25A_101 / SHA32A_101` по суставам.

---

## 4. Как посмотреть модель в MuJoCo

Самый простой вариант — использовать встроенный viewer MuJoCo:

1. Активировать окружение (если нужно):

   ```bash
   cd 10th_lecture
   source .venv/bin/activate
   ```

2. Запустить viewer с нужной моделью, например:

   ```bash
   mjpython -m mujoco.viewer --model ur_description/urdf/ur3_hd_replace_motor_single.xml
   ```

Или открыть `mujoco.viewer` отдельно и через меню  
**File → Open** выбрать нужный XML (`ur3_hd_replace_motor_single.xml`
или любой из `generated_models`).

Так можно сравнивать:

- исходный `mjmodel.xml` — «голый» UR3 без замены моторов;
- и `ur3_hd_replace_motor_single.xml` — UR3 с Harmonic Drive и обновлённой динамикой
  (массы, инерции, armature, передаточное число 101:1 и ограничения по моменту).
