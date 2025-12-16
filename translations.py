# translations.py
# Модуль для локализации и перевода текстов аддона

import bpy

TRANSLATIONS = {
    # Основные элементы UI
    "Generate Animation": {
        "EN": "Generate Animation",
        "RU": "Сгенерировать анимацию", 
        "DE": "Animation erzeugen",
        "ZH": "生成动画",
        "FR": "Générer l'animation",
        "ES": "Generar animación",
        "PT": "Gerar animação",
        "JA": "アニメーション生成",
        "KO": "애니메이션 생성"
    },
    "Export Preset": {
        "EN": "Export Preset",
        "RU": "Экспорт пресета",
        "DE": "Preset exportieren",
        "ZH": "导出预设",
        "FR": "Exporter préréglage",
        "ES": "Exportar preajuste",
        "PT": "Exportar predefinição",
        "JA": "プリセットエクスポート",
        "KO": "프리셋 내보내기"
    },
    "Import Preset": {
        "EN": "Import Preset",
        "RU": "Импорт пресета",
        "DE": "Preset importieren",
        "ZH": "导入预设",
        "FR": "Importer préréglage",
        "ES": "Importar preajuste",
        "PT": "Importar predefinição",
        "JA": "プリセットインポート",
        "KO": "프리셋 가져오기"
    },
    "Animation Type": {
        "EN": "Animation Type",
        "RU": "Тип анимации",
        "DE": "Animationstyp",
        "ZH": "动画类型",
        "FR": "Type d'animation",
        "ES": "Tipo de animación",
        "PT": "Tipo de animação",
        "JA": "アニメーションタイプ",
        "KO": "애니メ이션 유형"
    },
    
    # Типы анимаций
    "WALK": {
        "EN": "Walk",
        "RU": "Ходьба",
        "DE": "Gehen",
        "ZH": "行走",
        "FR": "Marcher",
        "ES": "Caminar",
        "PT": "Andar",
        "JA": "歩行",
        "KO": "걷기"
    },
    "TURN": {
        "EN": "Turn",
        "RU": "Поворот",
        "DE": "Drehung",
        "ZH": "转动",
        "FR": "Tourner",
        "ES": "Girar",
        "PT": "Virar",
        "JA": "回転",
        "KO": "회전"
    },
    "FULL BODY SWING": {
        "EN": "Full Body Swing",
        "RU": "Разворот всего тела",
        "DE": "Ganzkörper-Schwung",
        "ZH": "全身摆动",
        "FR": "Swing du corps entier",
        "ES": "Giro de cuerpo completo",
        "PT": "Balanço de corpo inteiro",
        "JA": "全身スイング",
        "KO": "전신 스윙"
    },
    "JUMP": {
        "EN": "Jump",
        "RU": "Прыжок",
        "DE": "Springen",
        "ZH": "跳跃",
        "FR": "Sauter",
        "ES": "Saltar",
        "PT": "Pular",
        "JA": "ジャンプ",
        "KO": "점프"
    },
    "IDLE": {
        "EN": "Idle",
        "RU": "Бездействие",
        "DE": "Leerlauf",
        "ZH": "待机",
        "FR": "Inactif",
        "ES": "Inactivo",
        "PT": "Ocioso",
        "JA": "待機",
        "KO": "대기"
    },
    "DRONE": {
        "EN": "Drone",
        "RU": "Дрон",
        "DE": "Drohne",
        "ZH": "无人机",
        "FR": "Drone",
        "ES": "Dron",
        "PT": "Drone",
        "JA": "ドローン",
        "KO": "드론"
    },
    "ATTACK": {
        "EN": "Attack",
        "RU": "Атака",
        "DE": "Angriff",
        "ZH": "攻击",
        "FR": "Attaque",
        "ES": "Ataque",
        "PT": "Ataque",
        "JA": "攻撃",
        "KO": "공격"
    },
    "CUSTOM": {
        "EN": "Custom",
        "RU": "Пользовательский",
        "DE": "Benutzerdefiniert",
        "ZH": "自定义",
        "FR": "Personnalisé",
        "ES": "Personalizado",
        "PT": "Personalizado",
        "JA": "カスタム",
        "KO": "사용자 정의"
    },
    
    # Настройки
    "Preset": {
        "EN": "Preset",
        "RU": "Пресет",
        "DE": "Preset",
        "ZH": "预设",
        "FR": "Préréglage",
        "ES": "Preajuste",
        "PT": "Predefinição",
        "JA": "プリセット",
        "KO": "프리셋"
    },
    "Generator Settings": {
        "EN": "Generator Settings",
        "RU": "Настройки генератора",
        "DE": "Generator-Einstellungen",
        "ZH": "生成器设置",
        "FR": "Paramètres du générateur",
        "ES": "Configuración del generador",
        "PT": "Configurações do gerador",
        "JA": "ジェネレーター設定",
        "KO": "생성기 설정"
    },
    "Step Height": {
        "EN": "Step Height",
        "RU": "Высота шага",
        "DE": "Schritthöhe",
        "ZH": "步高",
        "FR": "Hauteur de pas",
        "ES": "Altura del paso",
        "PT": "Altura do passo",
        "JA": "ステップ高さ",
        "KO": "스텝 높이"
    },
    "Stride Angle": {
        "EN": "Stride Angle",
        "RU": "Угол шага",
        "DE": "Schrittwinkel",
        "ZH": "步幅角度",
        "FR": "Angle de foulée",
        "ES": "Ángulo de zancada",
        "PT": "Ângulo da passada",
        "JA": "ストライド角度",
        "KO": "보폭 각도"
    },
    "Floatiness": {
        "EN": "Floatiness",
        "RU": "Плавучесть",
        "DE": "Schwebeeffekt",
        "ZH": "浮力感",
        "FR": "Flottabilité",
        "ES": "Flotabilidad",
        "PT": "Flutuabilidade",
        "JA": "浮遊感",
        "KO": "부양감"
    },
    "Orbital Motion": {
        "EN": "Orbital Motion",
        "RU": "Орбитальное движение",
        "DE": "Orbitalbewegung",
        "ZH": "轨道运动",
        "FR": "Mouvement orbital",
        "ES": "Movimiento orbital",
        "PT": "Movimento orbital",
        "JA": "軌道運動",
        "KO": "궤도 운동"
    },
    "Invert Joints": {
        "EN": "Invert Joints",
        "RU": "Инвертировать суставы",
        "DE": "Gelenke invertieren",
        "ZH": "反转关节",
        "FR": "Inverser les articulations",
        "ES": "Invertir articulaciones",
        "PT": "Inverter articulações",
        "JA": "ジョイント反転",
        "KO": "관절 반전"
    },
    "Lag / Delay": {
        "EN": "Lag / Delay",
        "RU": "Задержка",
        "DE": "Verzögerung",
        "ZH": "延迟",
        "FR": "Retard",
        "ES": "Retraso",
        "PT": "Atraso",
        "JA": "遅延",
        "KO": "지연"
    },
    "Noise Amount": {
        "EN": "Noise Amount",
        "RU": "Количество шума",
        "DE": "Rauschmenge",
        "ZH": "噪波量",
        "FR": "Quantité de bruit",
        "ES": "Cantidad de ruido",
        "PT": "Quantidade de ruído",
        "JA": "ノイズ量",
        "KO": "노이즈 양"
    },
    
    # Левая/правая нога
    "Leg Selection (overrides)": {
        "EN": "Leg Selection (overrides)",
        "RU": "Выбор ног (переопределения)",
        "DE": "Beinauswahl (Überschreibungen)",
        "ZH": "腿部选择（覆盖）",
        "FR": "Sélection des jambes (remplacements)",
        "ES": "Selección de piernas (anulaciones)",
        "PT": "Seleção de pernas (substituições)",
        "JA": "脚の選択（上書き）",
        "KO": "다리 선택 (재정의)"
    },
    "Left Bone (override)": {
        "EN": "Left Bone (override)",
        "RU": "Левая кость (переопределение)",
        "DE": "Linker Knochen (Überschreibung)",
        "ZH": "左骨骼（覆盖）",
        "FR": "Os gauche (remplacement)",
        "ES": "Hueso izquierdo (anulación)",
        "PT": "Osso esquerdo (substituição)",
        "JA": "左ボーン（上書き）",
        "KO": "왼쪽 뼈 (재정의)"
    },
    "Right Bone (override)": {
        "EN": "Right Bone (override)",
        "RU": "Правая кость (переопределение)",
        "DE": "Rechter Knochen (Überschreibung)",
        "ZH": "右骨骼（覆盖）",
        "FR": "Os droit (remplacement)",
        "ES": "Hueso derecho (anulación)",
        "PT": "Osso direito (substituição)",
        "JA": "右ボーン（上書き）",
        "KO": "오른쪽 뼈 (재정의)"
    },
    
    # COM / Центр масс
    "COM / Push": {
        "EN": "COM / Push",
        "RU": "Центр масс / Толчок",
        "DE": "COM / Schub",
        "ZH": "质心/推力",
        "FR": "COM / Poussée",
        "ES": "COM / Empuje",
        "PT": "COM / Empurrão",
        "JA": "COM/プッシュ",
        "KO": "COM/밀기"
    },
    "Push Strength": {
        "EN": "Push Strength",
        "RU": "Сила толчка",
        "DE": "Schubstärke",
        "ZH": "推力强度",
        "FR": "Force de poussée",
        "ES": "Fuerza de empuje",
        "PT": "Força de empurrão",
        "JA": "プッシュ強度",
        "KO": "밀기 강도"
    },
    "Push Profile": {
        "EN": "Push Profile",
        "RU": "Профиль толчка",
        "DE": "Schubprofil",
        "ZH": "推力曲线",
        "FR": "Profil de poussée",
        "ES": "Perfil de empuje",
        "PT": "Perfil de empurrão",
        "JA": "プッシュプロファイル",
        "KO": "밀기 프로필"
    },
    "Aggressiveness": {
        "EN": "Aggressiveness",
        "RU": "Агрессивность",
        "DE": "Aggressivität",
        "ZH": "攻击性",
        "FR": "Agressivité",
        "ES": "Agresividad",
        "PT": "Agressividade",
        "JA": "アグレッシブさ",
        "KO": "공격성"
    },
    "COM Inertia": {
        "EN": "COM Inertia",
        "RU": "Инерция центра масс",
        "DE": "COM-Trägheit",
        "ZH": "质心惯性",
        "FR": "Inertie du COM",
        "ES": "Inercia COM",
        "PT": "Inércia COM",
        "JA": "COM慣性",
        "KO": "COM 관성"
    },
    
    # Система координат
    "Coordinate System": {
        "EN": "Coordinate System",
        "RU": "Система координат",
        "DE": "Koordinatensystem",
        "ZH": "坐标系",
        "FR": "Système de coordonnées",
        "ES": "Sistema de coordenadas",
        "PT": "Sistema de coordenadas",
        "JA": "座標系",
        "KO": "좌표계"
    },
    "Forward Axis": {
        "EN": "Forward Axis",
        "RU": "Ось вперед",
        "DE": "Vorwärts-Achse",
        "ZH": "前进轴",
        "FR": "Axe avant",
        "ES": "Eje frontal",
        "PT": "Eixo frontal",
        "JA": "前方軸",
        "KO": "전방 축"
    },
    
    # Центральное движение
    "Forward / Center (COM)": {
        "EN": "Forward / Center (COM)",
        "RU": "Вперед / Центр (ЦМ)",
        "DE": "Vorwärts / Zentrum (COM)",
        "ZH": "前进/中心（质心）",
        "FR": "Avant / Centre (COM)",
        "ES": "Adelante / Centro (COM)",
        "PT": "Frente / Centro (COM)",
        "JA": "前方/中心（COM）",
        "KO": "전방/중심 (COM)"
    },
    "Enable Center COM": {
        "EN": "Enable Center COM",
        "RU": "Включить центр ЦМ",
        "DE": "COM-Zentrum aktivieren",
        "ZH": "启用质心中心",
        "FR": "Activer le centre COM",
        "ES": "Habilitar centro COM",
        "PT": "Habilitar centro COM",
        "JA": "COM中心を有効化",
        "KO": "COM 중심 활성화"
    },
    "Center Apply": {
        "EN": "Center Apply",
        "RU": "Применение центра",
        "DE": "Zentrum anwenden",
        "ZH": "中心应用",
        "FR": "Appliquer le centre",
        "ES": "Aplicar centro",
        "PT": "Aplicar centro",
        "JA": "中心適用",
        "KO": "중심 적용"
    },
    "Center Bone": {
        "EN": "Center Bone",
        "RU": "Центральная кость",
        "DE": "Zentralknochen",
        "ZH": "中心骨骼",
        "FR": "Os central",
        "ES": "Hueso central",
        "PT": "Osso central",
        "JA": "中心ボーン",
        "KO": "중심 뼈"
    },
    "Body Bob": {
        "EN": "Body Bob",
        "RU": "Покачивание тела",
        "DE": "Körperwippen",
        "ZH": "身体摆动",
        "FR": "Balancement du corps",
        "ES": "Balanceo corporal",
        "PT": "Balanço corporal",
        "JA": "体の揺れ",
        "KO": "몸 흔들림"
    },
    
    # Калибровка
    "Calibration": {
        "EN": "Calibration",
        "RU": "Калибровка",
        "DE": "Kalibrierung",
        "ZH": "校准",
        "FR": "Étalonnage",
        "ES": "Calibración",
        "PT": "Calibração",
        "JA": "キャリブレーション",
        "KO": "캘리브레이션"
    },
    "Enable Push Calibration": {
        "EN": "Enable Push Calibration",
        "RU": "Включить калибровку толчка",
        "DE": "Schubkalibrierung aktivieren",
        "ZH": "启用推力校准",
        "FR": "Activer l'étalonnage de poussée",
        "ES": "Habilitar calibración de empuje",
        "PT": "Habilitar calibração de empurrão",
        "JA": "プッシュキャリブレーションを有効化",
        "KO": "밀기 캘리브레이션 활성화"
    },
    
    # IK / Инверсная кинематика
    "IK / Legs": {
        "EN": "IK / Legs",
        "RU": "ИК / Ноги",
        "DE": "IK / Beine",
        "ZH": "逆运动学/腿部",
        "FR": "IK / Jambes",
        "ES": "IK / Piernas",
        "PT": "IK / Pernas",
        "JA": "IK/脚",
        "KO": "IK/다리"
    },
    "Use IK for Legs": {
        "EN": "Use IK for Legs",
        "RU": "Использовать ИК для ног",
        "DE": "IK für Beine verwenden",
        "ZH": "对腿部使用逆运动学",
        "FR": "Utiliser l'IK pour les jambes",
        "ES": "Usar IK para piernas",
        "PT": "Usar IK para pernas",
        "JA": "脚にIKを使用",
        "KO": "다리에 IK 사용"
    },
    "Rear Copy Mode": {
        "EN": "Rear Copy Mode",
        "RU": "Режим копирования задних",
        "DE": "Hintere Kopiermodus",
        "ZH": "后部复制模式",
        "FR": "Mode copie arrière",
        "ES": "Modo de copia trasera",
        "PT": "Modo de cópia traseira",
        "JA": "後部コピーモード",
        "KO": "후면 복사 모드"
    },
    
    # Фазирование
    "Phasing / Asymmetry": {
        "EN": "Phasing / Asymmetry",
        "RU": "Фазировка / Асимметрия",
        "DE": "Phasierung / Asymmetrie",
        "ZH": "相位/不对称",
        "FR": "Phasage / Asymétrie",
        "ES": "Fase / Asimetría",
        "PT": "Fase / Assimetria",
        "JA": "位相/非対称",
        "KO": "페이징/비대칭"
    },
    "Phase Offset": {
        "EN": "Phase Offset",
        "RU": "Смещение фазы",
        "DE": "Phasenversatz",
        "ZH": "相位偏移",
        "FR": "Décalage de phase",
        "ES": "Desplazamiento de fase",
        "PT": "Deslocamento de fase",
        "JA": "位相オフセット",
        "KO": "위상 오프셋"
    },
    "Phase Noise": {
        "EN": "Phase Noise",
        "RU": "Фазовый шум",
        "DE": "Phasenrauschen",
        "ZH": "相位噪波",
        "FR": "Bruit de phase",
        "ES": "Ruido de fase",
        "PT": "Ruído de fase",
        "JA": "位相ノイズ",
        "KO": "위상 노이즈"
    },
    "Phase Drift": {
        "EN": "Phase Drift",
        "RU": "Дрейф фазы",
        "DE": "Phasendrift",
        "ZH": "相位漂移",
        "FR": "Dérive de phase",
        "ES": "Deriva de fase",
        "PT": "Deriva de fase",
        "JA": "位相ドリフト",
        "KO": "위상 드리프트"
    },
    
    # Настройки поворота
    "Turn Settings": {
        "EN": "Turn Settings",
        "RU": "Настройки поворота",
        "DE": "Dreh-Einstellungen",
        "ZH": "转动设置",
        "FR": "Paramètres de rotation",
        "ES": "Configuración de giro",
        "PT": "Configurações de rotação",
        "JA": "回転設定",
        "KO": "회전 설정"
    },
    "Turn Angle (deg)": {
        "EN": "Turn Angle (deg)",
        "RU": "Угол поворота (град)",
        "DE": "Drehwinkel (Grad)",
        "ZH": "转动角度（度）",
        "FR": "Angle de rotation (degrés)",
        "ES": "Ángulo de giro (grados)",
        "PT": "Ângulo de rotação (graus)",
        "JA": "回転角度（度）",
        "KO": "회전 각도 (도)"
    },
    "Turn Speed": {
        "EN": "Turn Speed",
        "RU": "Скорость поворота",
        "DE": "Drehgeschwindigkeit",
        "ZH": "转动速度",
        "FR": "Vitesse de rotation",
        "ES": "Velocidad de giro",
        "PT": "Velocidade de rotação",
        "JA": "回転速度",
        "KO": "회전 속도"
    },
    "Spine Chain Settings": {
        "EN": "Spine Chain Settings",
        "RU": "Настройки цепи позвоночника",
        "DE": "Wirbelsäulenketten-Einstellungen",
        "ZH": "脊柱链设置",
        "FR": "Paramètres de la chaîne vertébrale",
        "ES": "Configuración de la cadena espinal",
        "PT": "Configurações da cadeia espinal",
        "JA": "脊柱チェーン設定",
        "KO": "척추 체인 설정"
    },
    "Spine Root Bone": {
        "EN": "Spine Root Bone",
        "RU": "Корневая кость позвоночника",
        "DE": "Wirbelsäulen-Wurzelknochen",
        "ZH": "脊柱根骨",
        "FR": "Os racine de la colonne",
        "ES": "Hueso raíz de la columna",
        "PT": "Osso raiz da coluna",
        "JA": "脊柱ルートボーン",
        "KO": "척추 루트 뼈"
    },
    "End Spine Bone": {
        "EN": "End Spine Bone",
        "RU": "Конечная кость позвоночника",
        "DE": "End-Wirbelsäulenknochen",
        "ZH": "脊柱末端骨",
        "FR": "Os de fin de colonne",
        "ES": "Hueso final de la columna",
        "PT": "Osso final da coluna",
        "JA": "脊柱エンドボーン",
        "KO": "척추 끝 뼈"
    },
    
    # Интерполяция поворота
    "Turn Interpolation": {
        "EN": "Turn Interpolation",
        "RU": "Интерполяция поворота",
        "DE": "Drehinterpolation",
        "ZH": "转动插值",
        "FR": "Interpolation de rotation",
        "ES": "Interpolación de giro",
        "PT": "Interpolação de rotação",
        "JA": "回転補間",
        "KO": "회전 보간"
    },
    "Interpolation Mode": {
        "EN": "Interpolation Mode",
        "RU": "Режим интерполяции",
        "DE": "Interpolationsmodus",
        "ZH": "插值模式",
        "FR": "Mode d'interpolation",
        "ES": "Modo de interpolación",
        "PT": "Modo de interpolação",
        "JA": "補間モード",
        "KO": "보간 모드"
    },
    
    # Настройки прыжка
    "Jump Settings": {
        "EN": "Jump Settings",
        "RU": "Настройки прыжка",
        "DE": "Spring-Einstellungen",
        "ZH": "跳跃设置",
        "FR": "Paramètres de saut",
        "ES": "Configuración de salto",
        "PT": "Configurações de salto",
        "JA": "ジャンプ設定",
        "KO": "점프 설정"
    },
    "Jump Height": {
        "EN": "Jump Height",
        "RU": "Высота прыжка",
        "DE": "Sprung höhe",
        "ZH": "跳跃高度",
        "FR": "Hauteur de saut",
        "ES": "Altura de salto",
        "PT": "Altura do salto",
        "JA": "ジャンプ高さ",
        "KO": "점프 높이"
    },
    "Jump Distance": {
        "EN": "Jump Distance",
        "RU": "Дистанция прыжка",
        "DE": "Sprungweite",
        "ZH": "跳跃距离",
        "FR": "Distance de saut",
        "ES": "Distancia de salto",
        "PT": "Distância do salto",
        "JA": "ジャンプ距離",
        "KO": "점프 거리"
    },
    "Crouch Factor": {
        "EN": "Crouch Factor",
        "RU": "Фактор приседания",
        "DE": "Hock-Faktor",
        "ZH": "下蹲因子",
        "FR": "Facteur d'accroupissement",
        "ES": "Factor de agachamiento",
        "PT": "Fator de agachamento",
        "JA": "しゃがみ係数",
        "KO": "웅크림 계수"
    },
    "Use IK for Legs": {
        "EN": "Use IK for Legs",
        "RU": "Использовать ИК для ног",
        "DE": "IK für Beine verwenden",
        "ZH": "腿部使用逆运动学",
        "FR": "Utiliser l'IK pour les jambes",
        "ES": "Usar IK para piernas",
        "PT": "Usar IK para pernas",
        "JA": "脚にIKを使用",
        "KO": "다리에 IK 사용"
    },
    "Arm Swing": {
        "EN": "Arm Swing",
        "RU": "Взмах рук",
        "DE": "Arm-Schwung",
        "ZH": "手臂摆动",
        "FR": "Balancement des bras",
        "ES": "Balanceo de brazos",
        "PT": "Balanço de braços",
        "JA": "腕の振り",
        "KO": "팔 스윙"
    },
    "Squash and Stretch": {
        "EN": "Squash and Stretch",
        "RU": "Сжатие и растяжение",
        "DE": "Quetschen und Strecken",
        "ZH": "挤压和拉伸",
        "FR": "Écrasement et étirement",
        "ES": "Aplastar y estirar",
        "PT": "Esmagar e esticar",
        "JA": "スカッシュ＆ストレッチ",
        "KO": "찌그러짐과 늘어남"
    },
    "Squash": {
        "EN": "Squash",
        "RU": "Сжатие",
        "DE": "Quetschen",
        "ZH": "挤压",
        "FR": "Écrasement",
        "ES": "Aplastar",
        "PT": "Esmagar",
        "JA": "スカッシュ",
        "KO": "찌그러짐"
    },
    "Stretch": {
        "EN": "Stretch",
        "RU": "Растяжение",
        "DE": "Strecken",
        "ZH": "拉伸",
        "FR": "Étirement",
        "ES": "Estirar",
        "PT": "Esticar",
        "JA": "ストレッチ",
        "KO": "늘어남"
    },
    "Trail Effect": {
        "EN": "Trail Effect",
        "RU": "Эффект шлейфа",
        "DE": "Schweifeffekt",
        "ZH": "拖尾效果",
        "FR": "Effet de traînée",
        "ES": "Efecto de estela",
        "PT": "Efeito de rastro",
        "JA": "トレイル効果",
        "KO": "궤적 효과"
    },
    "Secondary Motion": {
        "EN": "Secondary Motion",
        "RU": "Вторичное движение",
        "DE": "Sekundärbewegung",
        "ZH": "次要运动",
        "FR": "Mouvement secondaire",
        "ES": "Movimiento secundario",
        "PT": "Movimento secundário",
        "JA": "二次モーション",
        "KO": "보조 모션"
    },
    "Jump Phases": {
        "EN": "Jump Phases",
        "RU": "Фазы прыжка",
        "DE": "Sprungphasen",
        "ZH": "跳跃阶段",
        "FR": "Phases de saut",
        "ES": "Fases de salto",
        "PT": "Fases do salto",
        "JA": "ジャンプフェーズ",
        "KO": "점프 단계"
    },
    "Takeoff Duration": {
        "EN": "Takeoff Duration",
        "RU": "Длительность взлета",
        "DE": "Abflugdauer",
        "ZH": "起飞持续时间",
        "FR": "Durée de décollage",
        "ES": "Duración del despegue",
        "PT": "Duração da decolagem",
        "JA": "離陸期間",
        "KO": "이륙 기간"
    },
    "Hang Time": {
        "EN": "Hang Time",
        "RU": "Время зависания",
        "DE": "Schwebzeit",
        "ZH": "悬停时间",
        "FR": "Temps de suspension",
        "ES": "Tiempo de suspensión",
        "PT": "Tempo de suspensão",
        "JA": "ハングタイム",
        "KO": "공중 시간"
    },
    "Generate Jump Animation": {
        "EN": "Generate Jump Animation",
        "RU": "Сгенерировать анимацию прыжка",
        "DE": "Spring-Animation erzeugen",
        "ZH": "生成跳跃动画",
        "FR": "Générer l'animation de saut",
        "ES": "Generar animación de salto",
        "PT": "Gerar animação de salto",
        "JA": "ジャンプアニメーション生成",
        "KO": "점프 애니메이션 생성"
    },
    
    # Простые сообщения
    "Select an armature object": {
        "EN": "Select an armature object",
        "RU": "Выберите объект арматуры",
        "DE": "Wählen Sie ein Armature-Objekt",
        "ZH": "请选择一个骨架对象",
        "FR": "Sélectionnez un objet armature",
        "ES": "Seleccione un objeto de armadura",
        "PT": "Selecione um objeto de armadura",
        "JA": "アーマチュアオブジェクトを選択してください",
        "KO": "아머처 객체를 선택하세요"
    },
    "Generated animation for {left}, {right}": {
        "EN": "Generated animation for {left}, {right}",
        "RU": "Сгенерирована анимация для {left}, {right}",
        "DE": "Animation erstellt für {left}, {right}",
        "ZH": "已为 {left}, {right} 生成动画",
        "FR": "Animation générée pour {left}, {right}",
        "ES": "Animación generada para {left}, {right}",
        "PT": "Animação gerada para {left}, {right}",
        "JA": "{left}、{right} のアニメーションを生成しました",
        "KO": "{left}, {right}에 대한 애니메이션 생성됨"
    },
    "Generated turn animation for spine chain from {root} to {end}": {
        "EN": "Generated turn animation for spine chain from {root} to {end}",
        "RU": "Сгенерирована анимация поворота для цепочки позвоночника от {root} до {end}",
        "DE": "Drehanimation für Wirbelsäulenkette von {root} bis {end} erstellt",
        "ZH": "已为从 {root} 到 {end} 的脊柱链生成转动动画",
        "FR": "Animation de rotation générée pour la chaîne vertébrale de {root} à {end}",
        "ES": "Animación de giro generada para la cadena espinal desde {root} hasta {end}",
        "PT": "Animação de rotação gerada para a cadeia espinal de {root} a {end}",
        "JA": "脊柱チェーン {root} から {end} への回転アニメーションを生成しました",
        "KO": "{root}에서 {end}까지의 척추 체인 회전 애니메이션 생성됨"
    }
}

def tr(key, lang='AUTO'):
    """
    Функция перевода текста на основе текущего языка интерфейса Blender
    """
    code = lang
    if code == 'AUTO':
        try:
            ui_lang = bpy.context.preferences.view.language
        except Exception:
            ui_lang = 'DEFAULT'
        if ui_lang is None or ui_lang == 'DEFAULT':
            code = 'EN'
        else:
            ui_lang = ui_lang.lower()
            if ui_lang.startswith('ru'): code = 'RU'
            elif ui_lang.startswith('de'): code = 'DE'
            elif ui_lang.startswith('zh'): code = 'ZH'
            elif ui_lang.startswith('fr'): code = 'FR'
            elif ui_lang.startswith('es'): code = 'ES'
            elif ui_lang.startswith('pt'): code = 'PT'
            elif ui_lang.startswith('ja'): code = 'JA'
            elif ui_lang.startswith('ko'): code = 'KO'
            else: code = 'EN'
    
    if key not in TRANSLATIONS:
        return key
    
    return TRANSLATIONS[key].get(code, TRANSLATIONS[key].get('EN', key))

def register():
    """Регистрация модуля переводов"""
    pass

def unregister():
    """Снятие регистрации модуля переводов"""
    pass