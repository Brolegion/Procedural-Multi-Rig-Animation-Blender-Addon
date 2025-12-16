# noise_provider.py
import sys
import math
import random
from types import SimpleNamespace

# Попытка импортов — не фатально, определим поддержку динамически
_has_opensimplex = False
_has_noise = False
_has_mathutils = False
_opensimplex = None
_noise = None
_mathutils_noise = None

try:
    from opensimplex import OpenSimplex
    _has_opensimplex = True
    _opensimplex = OpenSimplex
except Exception:
    _has_opensimplex = False

try:
    # пакет 'noise' предоставляет pnoise2/pnoise3/snoise* (Perlin/Simplex)
    import noise as _noise_mod
    _has_noise = True
    _noise = _noise_mod
except Exception:
    _has_noise = False

# mathutils.noise доступен только внутри Blender
try:
    import bpy
    from mathutils import noise as _mnoise
    _has_mathutils = True
    _mathutils_noise = _mnoise
except Exception:
    _has_mathutils = False

# Fallback deterministic sum-of-sines helper (seedable)
def _seeded_sines_series(seed, octaves=5, base_freq=1.0, freq_spread=2.0):
    rnd = random.Random(seed)
    terms = []
    amp = 1.0
    for i in range(octaves):
        a = amp * (0.5 ** i)
        f = base_freq * (freq_spread ** i) * (0.9 + rnd.random() * 0.2)
        p = rnd.random() * math.pi * 2.0
        terms.append((a, f, p))
    # normalize total amplitude
    total = sum(abs(t[0]) for t in terms) or 1.0
    return terms, total

def _seeded_sines_eval(terms, total_amp, t):
    v = 0.0
    for a, f, p in terms:
        v += a * math.sin(t * f + p)
    return v / total_amp

# =============================================
# УЛУЧШЕННЫЙ SINES FALLBACK (почти как Perlin)
# =============================================
class _ImprovedSinesNoise:
    """Улучшенный шум на основе синусов с качеством близким к Perlin"""
    
    def __init__(self, seed):
        self.seed = seed
        self.rnd = random.Random(seed)
        # Создаем несколько наборов синусов для разных измерений
        self.terms_1d = self._create_terms(octaves=6, base_freq=1.0)
        self.terms_2d_x = self._create_terms(octaves=6, base_freq=0.7)
        self.terms_2d_y = self._create_terms(octaves=6, base_freq=0.9)
        self.terms_3d = self._create_terms(octaves=5, base_freq=0.5)
    
    def _create_terms(self, octaves=5, base_freq=1.0):
        terms = []
        amp = 1.0
        for i in range(octaves):
            a = amp * (0.6 ** i)  # Более плавный спад
            f = base_freq * (2.0 ** i) * (0.8 + self.rnd.random() * 0.4)
            p = self.rnd.random() * math.pi * 2.0
            terms.append((a, f, p))
        total = sum(abs(t[0]) for t in terms) or 1.0
        return terms, total
    
    def noise1d(self, x):
        terms, total = self.terms_1d
        # Добавляем несколько слоев для более интересного шума
        n1 = _seeded_sines_eval(terms, total, x)
        n2 = _seeded_sines_eval(terms, total, x * 1.7 + 10.0) * 0.3
        n3 = _seeded_sines_eval(terms, total, x * 3.5 + 20.0) * 0.1
        return (n1 + n2 + n3) / 1.4
    
    def noise2d(self, x, y):
        terms_x, total_x = self.terms_2d_x
        terms_y, total_y = self.terms_2d_y
        
        # Косоугольные комбинации для более интересных паттернов
        nx = _seeded_sines_eval(terms_x, total_x, x + y * 0.3)
        ny = _seeded_sines_eval(terms_y, total_y, y + x * 0.7)
        nxy = _seeded_sines_eval(terms_x, total_x, x * 0.8 - y * 0.6) * 0.5
        
        return (nx + ny + nxy) / 2.5
    
    def noise3d(self, x, y, z):
        terms, total = self.terms_3d
        # Используем 3D с комбинацией координат
        nx = _seeded_sines_eval(terms, total, x + y * 0.3 + z * 0.1)
        ny = _seeded_sines_eval(terms, total, y + z * 0.3 + x * 0.1)
        nz = _seeded_sines_eval(terms, total, z + x * 0.3 + y * 0.1)
        return (nx + ny + nz) / 3.0

# =============================================
# ОСНОВНОЙ КЛАСС NOISE PROVIDER
# =============================================
class NoiseProvider:
    """
    Унифицированный интерфейс к шумовым бэкендам.
    Приоритет: OpenSimplex > noise module > mathutils > improved sines
    """
    
    def __init__(self, seed=0, prefer=None, allow_mathutils=True):
        """
        seed: int - всегда работает (для mathutils используется как смещение)
        prefer: optional str to force backend: 'opensimplex', 'noise', 'mathutils', 'sines'
        allow_mathutils: bool - если False, пропускаем mathutils даже если доступен
        """
        self.seed = int(seed or 0)
        self.backend = None
        self.backend_name = None
        self._opensimplex = None
        self._noise = None
        self._mathutils = None
        self._improved_sines = None
        self._uses_seed = True
        self._mathutils_offset = Vector((0, 0, 0)) if _has_mathutils else None
        
        # Настраиваем смещения для mathutils на основе seed
        if _has_mathutils:
            # Используем seed для создания уникальных смещений
            self._mathutils_offset = Vector((
                (seed * 1000.0) % 10000.0,
                (seed * 2000.0) % 10000.0,
                (seed * 3000.0) % 10000.0
            ))
        
        # Выбор бэкенда
        pref = (prefer or "").lower()
        
        # Если указано предпочтение
        if pref == "opensimplex" and _has_opensimplex:
            self._use_opensimplex()
        elif pref == "noise" and _has_noise:
            self._use_noise()
        elif pref == "mathutils" and _has_mathutils and allow_mathutils:
            self._use_mathutils()
        elif pref == "sines":
            self._use_improved_sines()
        else:
            # Автовыбор лучшего доступного
            if _has_opensimplex:
                self._use_opensimplex()
            elif _has_noise:
                self._use_noise()
            elif _has_mathutils and allow_mathutils:
                self._use_mathutils()
            else:
                self._use_improved_sines()
    
    def available_backends(self):
        """Возвращает список доступных бэкендов"""
        flags = {
            "opensimplex": _has_opensimplex,
            "noise_module": _has_noise,
            "mathutils_noise": _has_mathutils,
            "improved_sines": True  # Всегда доступен
        }
        return flags
    
    # ==================== БЭКЕНДЫ ====================
    
    def _use_opensimplex(self):
        """OpenSimplex - лучший вариант"""
        self.backend = "opensimplex"
        self.backend_name = "OpenSimplex"
        self._opensimplex = _opensimplex(seed=self.seed)
        self._uses_seed = True
    
    def _use_noise(self):
        """Модуль noise (Perlin/Simplex)"""
        self.backend = "noise"
        self.backend_name = "noise module"
        self._noise = _noise
        # Устанавливаем seed для модуля noise
        import random
        random.seed(self.seed)
        self._uses_seed = True
    
    def _use_mathutils(self):
        """mathutils.noise - самый быстрый, но с ограничениями"""
        self.backend = "mathutils"
        self.backend_name = "mathutils.noise (fast)"
        self._mathutils = _mathutils_noise
        self._uses_seed = False  # Реальный seed не поддерживается
    
    def _use_improved_sines(self):
        """Улучшенный синусоидальный шум"""
        self.backend = "sines"
        self.backend_name = "improved sines"
        self._improved_sines = _ImprovedSinesNoise(self.seed)
        self._uses_seed = True
    
    # ==================== ОСНОВНЫЕ ФУНКЦИИ ШУМА ====================
    
    def noise1d(self, x, scale=1.0):
        """1D шум ~[-1..1]"""
        xs = x * scale
        
        if self.backend == "opensimplex":
            return self._opensimplex.noise2d(xs, 0.0)
        
        elif self.backend == "noise":
            return self._noise.snoise2(xs, 0.0)
        
        elif self.backend == "mathutils":
            # Используем смещение для эмуляции seed
            if self._mathutils_offset:
                return self._mathutils.noise((xs + self._mathutils_offset.x, 0.0, 0.0))
            return self._mathutils.noise((xs, 0.0, 0.0))
        
        else:  # sines
            return self._improved_sines.noise1d(xs)
    
    def noise2d(self, x, y, scale=1.0):
        """2D шум ~[-1..1]"""
        xs, ys = x * scale, y * scale
        
        if self.backend == "opensimplex":
            return self._opensimplex.noise2d(xs, ys)
        
        elif self.backend == "noise":
            return self._noise.snoise2(xs, ys)
        
        elif self.backend == "mathutils":
            if self._mathutils_offset:
                return self._mathutils.noise((xs + self._mathutils_offset.x, ys + self._mathutils_offset.y, 0.0))
            return self._mathutils.noise((xs, ys, 0.0))
        
        else:  # sines
            return self._improved_sines.noise2d(xs, ys)
    
    def noise3d(self, x, y, z, scale=1.0):
        """3D шум ~[-1..1]"""
        xs, ys, zs = x * scale, y * scale, z * scale
        
        if self.backend == "opensimplex":
            return self._opensimplex.noise3d(xs, ys, zs)
        
        elif self.backend == "noise":
            return self._noise.snoise3(xs, ys, zs)
        
        elif self.backend == "mathutils":
            if self._mathutils_offset:
                return self._mathutils.noise((xs + self._mathutils_offset.x, ys + self._mathutils_offset.y, zs + self._mathutils_offset.z))
            return self._mathutils.noise((xs, ys, zs))
        
        else:  # sines
            return self._improved_sines.noise3d(xs, ys, zs)
    
    # ==================== РАСШИРЕННЫЕ ФУНКЦИИ ====================
    
    def fbm1d(self, x, octaves=5, lacunarity=2.0, gain=0.5, scale=1.0):
        """Fractal Brownian Motion (1D)"""
        amp = 1.0
        freq = 1.0
        s = 0.0
        max_a = 0.0
        
        for _ in range(octaves):
            s += amp * self.noise1d(x * freq, scale=scale)
            max_a += amp
            amp *= gain
            freq *= lacunarity
        
        return s / max_a if max_a else 0.0
    
    def fbm2d(self, x, y, octaves=5, lacunarity=2.0, gain=0.5, scale=1.0):
        """Fractal Brownian Motion (2D)"""
        amp = 1.0
        freq = 1.0
        s = 0.0
        max_a = 0.0
        
        for _ in range(octaves):
            s += amp * self.noise2d(x * freq, y * freq, scale=scale)
            max_a += amp
            amp *= gain
            freq *= lacunarity
        
        return s / max_a if max_a else 0.0
    
    def fbm3d(self, x, y, z, octaves=5, lacunarity=2.0, gain=0.5, scale=1.0):
        """Fractal Brownian Motion (3D)"""
        amp = 1.0
        freq = 1.0
        s = 0.0
        max_a = 0.0
        
        for _ in range(octaves):
            s += amp * self.noise3d(x * freq, y * freq, z * freq, scale=scale)
            max_a += amp
            amp *= gain
            freq *= lacunarity
        
        return s / max_a if max_a else 0.0
    
    def curl3d(self, x, y, z, eps=1e-3, scale=1.0):
        """Curl noise (3D) - возвращает нормализованный вектор"""
        # Создаем 3 скалярных поля с разными смещениями
        def F(px, py, pz):
            return (
                self.noise3d(px + 12.345, py, pz, scale=scale),
                self.noise3d(px, py + 34.567, pz, scale=scale),
                self.noise3d(px, py, pz + 78.901, scale=scale)
            )
        
        # Центральные разности
        fxp = F(x + eps, y, z)
        fxm = F(x - eps, y, z)
        fyp = F(x, y + eps, z)
        fym = F(x, y - eps, z)
        fzp = F(x, y, z + eps)
        fzm = F(x, y, z - eps)
        
        # Частные производные
        dFdx = tuple((fxp[i] - fxm[i]) / (2 * eps) for i in range(3))
        dFdy = tuple((fyp[i] - fym[i]) / (2 * eps) for i in range(3))
        dFdz = tuple((fzp[i] - fzm[i]) / (2 * eps) for i in range(3))
        
        # Curl
        cx = dFdz[1] - dFdy[2]
        cy = dFdx[2] - dFdz[0]
        cz = dFdy[0] - dFdx[1]
        
        # Нормализация
        mag = math.sqrt(cx*cx + cy*cy + cz*cz) + 1e-12
        return (cx/mag, cy/mag, cz/mag)
    
    def turbulence(self, x, y, octaves=6):
        """Турбулентный шум (abs(noise))"""
        value = 0.0
        amp = 1.0
        freq = 1.0
        
        for _ in range(octaves):
            value += amp * abs(self.noise2d(x * freq, y * freq))
            amp *= 0.5
            freq *= 2.0
        
        return value
    
    # ==================== УТИЛИТЫ ====================
    
    def get_backend_info(self):
        """Информация о текущем бэкенде"""
        return {
            "backend": self.backend,
            "name": self.backend_name,
            "uses_seed": self._uses_seed,
            "seed": self.seed if self._uses_seed else "not supported"
        }
    
    def set_seed(self, new_seed):
        """Установить новый seed (работает только если бэкенд поддерживает)"""
        self.seed = int(new_seed)
        
        if self.backend == "opensimplex":
            self._opensimplex = _opensimplex(seed=self.seed)
        
        elif self.backend == "noise":
            import random
            random.seed(self.seed)
        
        elif self.backend == "sines":
            self._improved_sines = _ImprovedSinesNoise(self.seed)
        
        elif self.backend == "mathutils" and _has_mathutils:
            # Обновляем смещение
            self._mathutils_offset = Vector((
                (self.seed * 1000.0) % 10000.0,
                (self.seed * 2000.0) % 10000.0,
                (self.seed * 3000.0) % 10000.0
            ))
    
    def __repr__(self):
        info = self.get_backend_info()
        return f"<NoiseProvider: {info['name']}, seed={info['seed']}>"


# =============================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# =============================================
def get_default_noise_provider(seed=0):
    """
    Создает NoiseProvider с оптимальными настройками по умолчанию
    для анимации в Blender.
    """
    return NoiseProvider(
        seed=seed,
        prefer="opensimplex" if _has_opensimplex else "sines",
        allow_mathutils=True
    )


def debug_noise_quality(seed=12345, samples=10):
    """
    Тестирует качество шума для разных бэкендов
    """
    backends_to_test = []
    
    if _has_opensimplex:
        backends_to_test.append(("OpenSimplex", "opensimplex"))
    if _has_noise:
        backends_to_test.append(("Noise module", "noise"))
    if _has_mathutils:
        backends_to_test.append(("MathUtils", "mathutils"))
    backends_to_test.append(("Improved Sines", "sines"))
    
    print("=" * 50)
    print("NOISE PROVIDER QUALITY TEST")
    print("=" * 50)
    
    for name, pref in backends_to_test:
        try:
            np = NoiseProvider(seed=seed, prefer=pref)
            print(f"\n{name}:")
            print(f"  Backend: {np.backend_name}")
            print(f"  Seed supported: {np._uses_seed}")
            
            # Тест случайности
            values = []
            for i in range(samples):
                values.append(np.noise1d(i * 0.1))
            
            avg = sum(values) / len(values)
            variance = sum((v - avg) ** 2 for v in values) / len(values)
            
            print(f"  Avg: {avg:.4f} (should be ~0)")
            print(f"  Variance: {variance:.4f} (should be ~0.3)")
            print(f"  Range: {min(values):.4f} to {max(values):.4f}")
            
        except Exception as e:
            print(f"\n{name}: ERROR - {e}")
    
    print("\n" + "=" * 50)
    print("RECOMMENDATION:")
    if _has_opensimplex:
        print("✓ Use OpenSimplex (pip install opensimplex)")
    elif _has_noise:
        print("✓ Use noise module (already installed)")
    elif _has_mathutils:
        print("✓ Use mathutils.noise (fast, but limited seed support)")
    else:
        print("✓ Use improved sines (always available)")
    print("=" * 50)


def smooth_noise1d(self, x, octaves=3, persistence=0.5, lacunarity=2.0, scale=1.0):
    """Гладкий 1D шум с октавами для более натурального движения"""
    total = 0.0
    amplitude = 1.0
    frequency = 1.0
    max_value = 0.0

    for i in range(octaves):
        total += self.noise1d(x * frequency, scale=scale) * amplitude
        max_value += amplitude
        amplitude *= persistence
        frequency *= lacunarity

    return total / max_value if max_value > 0 else 0.0

# Импорт Vector для mathutils
try:
    from mathutils import Vector
except ImportError:
    # Fallback для тестов вне Blender
    class Vector:
        def __init__(self, values):
            self.x, self.y, self.z = values[0], values[1], values[2] if len(values) > 2 else 0.0