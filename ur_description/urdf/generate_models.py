# generate_models.py
from dataclasses import dataclass
from copy import deepcopy
from itertools import product
from pathlib import Path

import numpy as np
from lxml import etree

from harmonic_drives import HarmonicDrive, joint_variants
# joint_variants: словарь joint -> список HarmonicDrive (SHA20/25/32)


# ========= Базовые функции работы с XML =========

def load_xml(path: str) -> etree._ElementTree:
    return etree.parse(path)


def save_xml(tree: etree._ElementTree, path: str) -> None:
    tree.write(
        path,
        pretty_print=True,
        xml_declaration=True,
        encoding="utf-8",
    )


def find_joints(tree: etree._ElementTree):
    root = tree.getroot()
    return root.findall(".//joint")


def map_joint_to_body(tree: etree._ElementTree):
    """
    joint_name -> (joint_element, parent_body_element)
    (joint лежит внутри body, к которому он прикреплён).
    """
    res = {}
    for joint in find_joints(tree):
        name = joint.get("name")
        if not name:
            continue
        body = joint.getparent()
        res[name] = (joint, body)
    return res


def get_or_create_actuator_root(root: etree._Element) -> etree._Element:
    act = root.find("actuator")
    if act is None:
        act = etree.SubElement(root, "actuator")
    return act


# ========= Работа с <inertial> =========

def get_inertial(body_el: etree._Element) -> etree._Element:
    inertial = body_el.find("inertial")
    if inertial is None:
        raise RuntimeError(f"No <inertial> in body '{body_el.get('name')}'")
    return inertial


def parse_inertial(inertial_el: etree._Element):
    m = float(inertial_el.get("mass"))
    pos_str = inertial_el.get("pos", "0 0 0")
    pos = np.fromstring(pos_str, sep=" ")
    I_diag = np.fromstring(inertial_el.get("diaginertia"), sep=" ")
    return m, pos, I_diag


def cylinder_inertia_diag(m: float, R: float, h: float) -> np.ndarray:
    """
    Диагональ тензора инерции цилиндра в его центре масс,
    ось цилиндра вдоль локальной Z.
    """
    i_xy = (1.0 / 12.0) * m * (3 * R**2 + h**2)
    i_z = 0.5 * m * R**2
    return np.array([i_xy, i_xy, i_z])


# ========= armature (инерция по оси сустава) =========

def set_joint_armature(joint_el: etree._Element, hd: HarmonicDrive) -> None:
    """
    armature в MuJoCo — инерция по оси сустава.
    В hd.armature_inertia хранится колонка GD^2/4 (Moment of Inertia),
    то есть уже инерция актуатора на стороне сустава → просто пишем её.
    """
    joint_el.set("armature", f"{hd.armature_inertia:.6e}")


# ========= Описание «старых» моторов UR3 =========

@dataclass
class MotorShell:
    mass: float   # kg
    radius: float # m
    length: float # m


OLD_MOTORS = {
    "shoulder_pan_joint": MotorShell(mass=1.0,  radius=0.04,  length=0.08),
    "shoulder_lift_joint":MotorShell(mass=1.0,  radius=0.04,  length=0.08),
    "elbow_joint":        MotorShell(mass=0.6,  radius=0.035, length=0.07),
    "wrist_1_joint":      MotorShell(mass=0.3,  radius=0.025, length=0.05),
    "wrist_2_joint":      MotorShell(mass=0.3,  radius=0.025, length=0.05),
    "wrist_3_joint":      MotorShell(mass=0.15, radius=0.020, length=0.04),
}


def replace_motor_in_inertial(
    inertial_el: etree._Element,
    old_m: MotorShell,
    new_hd: HarmonicDrive,
) -> None:
    """
    Заменяет в инерции звена старый мотор на новый SHA-SG.

    Допущения:
      - текущий <inertial> описывает (звено + старый мотор);
      - COM старого и нового мотора совпадает с pos inertial;
      - оси цилиндров совпадают с осями инерциальной рамки.
    Тогда Штейнер упрощается, и мы работаем только с диагоналями.
    """
    m_tot, pos, I_tot = parse_inertial(inertial_el)

    # 1) вычитаем старый цилиндр
    I_old = cylinder_inertia_diag(old_m.mass, old_m.radius, old_m.length)
    m_clean = m_tot - old_m.mass
    if m_clean <= 0:
        raise RuntimeError(
            f"Non-positive clean mass for body '{inertial_el.getparent().get('name')}'"
        )
    I_clean = I_tot - I_old

    # 2) добавляем новый цилиндр SHA-SG
    I_new = cylinder_inertia_diag(new_hd.mass, new_hd.radius, new_hd.length)
    m_final = m_clean + new_hd.mass
    I_final = I_clean + I_new

    inertial_el.set("mass", f"{m_final:.6e}")
    inertial_el.set("pos", " ".join(f"{v:.6e}" for v in pos))
    inertial_el.set("diaginertia", " ".join(f"{v:.6e}" for v in I_final))


def add_motor_actuator(actuator_root: etree._Element,
                       joint_name: str,
                       hd: HarmonicDrive) -> None:
    """
    Добавляет general-актуатор с ограничением по моменту:
      ctrlrange = ± Maximum Torque из каталога SHA-SG.
    """
    motor_name = f"{joint_name}_motor_{hd.name}"
    T_max = hd.max_torque  # N·m

    motor_el = etree.SubElement(actuator_root, "general")
    motor_el.set("name", motor_name)
    motor_el.set("joint", joint_name)
    motor_el.set("gear", "1.0")  # управляющий сигнал = момент в суставе
    motor_el.set("ctrlrange", f"{-T_max:.3f} {T_max:.3f}")

def replace_motor_in_inertial_ur3(
    inertial_el: etree._Element,
    old_motor: MotorShell,
    new_hd: HarmonicDrive,
) -> None:
    _, r_t, _ = parse_inertial(inertial_el)

    replace_motor_in_inertial_full(
        inertial_el=inertial_el,
        old_motor=old_motor,
        new_motor=new_hd,
        r_m_old=np.array(r_t),    # мотор сидит в COM звена
        R_old=np.eye(3),          # оси совпадают
        r_m_new=np.array(r_t),    # новый мотор тоже туда же
        R_new=np.eye(3),
    )

# ========= Построение одной модели (базовый набор SHA-SG) =========

def build_one_variant(base_xml_path: str, output_path: str) -> None:
    """
    Делает одну модель UR3 с конкретным набором SHA-SG
    (берём первый элемент из списков joint_variants).
    """
    tree = load_xml(base_xml_path)
    root = tree.getroot()

    joint_map = map_joint_to_body(tree)
    actuator_root = get_or_create_actuator_root(root)

    for joint_name, hd_list in joint_variants.items():
        hd = hd_list[0]  # в базовом варианте по одному на сустав
        old_motor = OLD_MOTORS[joint_name]

        joint_el, body_el = joint_map[joint_name]
        inertial_el = get_inertial(body_el)

        set_joint_armature(joint_el, hd)
        replace_motor_in_inertial_ur3(inertial_el, old_motor, hd)
        add_motor_actuator(actuator_root, joint_name, hd)

    save_xml(tree, output_path)
    print(f"Saved base variant to: {output_path}")



def generate_all_variants(base_xml_path: str, output_dir: str) -> None:
    """
    Для каждого сустава перебирает все варианты HarmonicDrive из joint_variants
    и генерирует отдельный MJCF-файл.

    shoulder_pan / shoulder_lift – обычно только SHA32A-101,
    elbow – SHA25A-101 или SHA32A-101,
    wrists – SHA20A-101 или SHA25A-101.
    """
    base_tree = load_xml(base_xml_path)
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    variants_lists = [joint_variants[j] for j in joint_names]

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for idx, combo in enumerate(product(*variants_lists), start=1):
        tree = deepcopy(base_tree)
        root = tree.getroot()
        joint_map = map_joint_to_body(tree)
        actuator_root = get_or_create_actuator_root(root)

        # Применяем конкретный набор SHA-SG к суставам
        for joint_name, hd in zip(joint_names, combo):
            joint_el, body_el = joint_map[joint_name]
            inertial_el = get_inertial(body_el)
            set_joint_armature(joint_el, hd)
            replace_motor_in_inertial(inertial_el, OLD_MOTORS[joint_name], hd)
            add_motor_actuator(actuator_root, joint_name, hd)


        suffix = "_".join(f"{hd.name}" for hd in combo)
        out_path = out_dir / f"ur3_sha_sg_101_variant_{idx:02d}.xml"
        save_xml(tree, out_path)
        print(f"[{idx}] saved: {out_path}")



def replace_motor_in_inertial_full(
    inertial_el: etree._Element,
    old_motor,                      
    new_motor,                       
    r_m_old: np.ndarray | None = None,  # COM старого мотора в осях звена (3,)
    R_old: np.ndarray | None = None,    # 3x3: от главных осей мотора к осям звена
    r_m_new: np.ndarray | None = None,  # COM нового мотора в осях звена (3,)
    R_new: np.ndarray | None = None,    # 3x3 для нового мотора
) -> None:

    I3 = np.eye(3)

    M_tot, r_t, I_diag = parse_inertial(inertial_el)
    r_t = np.asarray(r_t, dtype=float)
    I_tot = np.diag(I_diag)  

    if old_motor is not None:
        m_old = float(old_motor.mass)

        M_clean = M_tot - m_old
        if M_clean <= 0:
            raise RuntimeError(
                f"Non-positive clean mass: {M_clean} for body {inertial_el.getparent().get('name')}"
            )


        if r_m_old is None:
            r_m_old = r_t.copy()
        else:
            r_m_old = np.asarray(r_m_old, dtype=float)

        if R_old is None:
            R_old = np.eye(3)
        else:
            R_old = np.asarray(R_old, dtype=float).reshape(3, 3)

        I_old_cm_diag = cylinder_inertia_diag(m_old, old_motor.radius, old_motor.length)
        I_old_cm = np.diag(I_old_cm_diag)
        I_old_aligned = R_old @ I_old_cm @ R_old.T

        d_old = r_m_old - r_t
        I_old_about_rt = I_old_aligned + m_old * (
            (np.dot(d_old, d_old)) * I3 - np.outer(d_old, d_old)
        )

        I_clean_about_rt = I_tot - I_old_about_rt


        r_clean = (M_tot * r_t - m_old * r_m_old) / M_clean

        d_shift = r_clean - r_t
        I_clean = I_clean_about_rt - M_clean * (
            (np.dot(d_shift, d_shift)) * I3 - np.outer(d_shift, d_shift)
        )
    else:

        M_clean = M_tot
        r_clean = r_t.copy()
        I_clean = I_tot


    if new_motor is not None:
        m_new = float(new_motor.mass)

        if r_m_new is None:
            r_m_new = r_clean.copy()
        else:
            r_m_new = np.asarray(r_m_new, dtype=float)

        if R_new is None:
            R_new = np.eye(3)
        else:
            R_new = np.asarray(R_new, dtype=float).reshape(3, 3)
        I_new_cm_diag = cylinder_inertia_diag(m_new, new_motor.radius, new_motor.length)
        I_new_cm = np.diag(I_new_cm_diag)

        I_new_aligned = R_new @ I_new_cm @ R_new.T

        d_new = r_m_new - r_clean
        I_new_about_r_clean = I_new_aligned + m_new * (
            (np.dot(d_new, d_new)) * I3 - np.outer(d_new, d_new)
        )

        M_final = M_clean + m_new
        I_tot_about_r_clean = I_clean + I_new_about_r_clean

        r_final = (M_clean * r_clean + m_new * r_m_new) / M_final

        d_shift2 = r_final - r_clean
        I_final = I_tot_about_r_clean - M_final * (
            (np.dot(d_shift2, d_shift2)) * I3 - np.outer(d_shift2, d_shift2)
        )
    else:
        M_final = M_clean
        r_final = r_clean
        I_final = I_clean


    # В общем случае I_final не обязательно диагонален.
    # MJCF позволяет хранить либо полную матрицу inertia, либо только diag.
    # Здесь берём только диагональ (если нужно – можно расширить до полноформатного inertia).
    I_final_diag = np.diag(I_final)

    inertial_el.set("mass", f"{M_final:.6e}")
    inertial_el.set("pos", " ".join(f"{v:.6e}" for v in r_final))
    inertial_el.set("diaginertia", " ".join(f"{v:.6e}" for v in I_final_diag))


if __name__ == "__main__":
    base_xml = "mjmodel.xml"  # исходный UR3 MJCF

    # одна базовая модель
    build_one_variant(base_xml, "ur3_hd_replace_motor_single.xml")

    # все комбинации SHA20/25/32
    generate_all_variants(base_xml, "generated_models")
