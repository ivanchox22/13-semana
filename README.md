# Control por Rechazo Activo de Perturbaciones (ADRC)

El Control por Rechazo Activo de Perturbaciones (ADRC, por sus siglas en inglés *Active Disturbance Rejection Control*) es una técnica avanzada de control que surge como una alternativa robusta al control PID tradicional. Desarrollado por el Dr. Zhiqiang Gao, el ADRC busca mitigar las limitaciones del PID, especialmente su dependencia de un modelo preciso de la planta y su sensibilidad a perturbaciones externas e incertidumbres internas. 

El ADRC se basa en tres pilares fundamentales:
1. **Estimación de perturbaciones**: Utiliza un Observador de Estados Extendido (ESO) para estimar y compensar perturbaciones en tiempo real.
2. **Independencia del modelo**: Requiere solo información básica del sistema, como su orden y ganancia nominal.
3. **Simplificación del diseño**: Transforma el sistema real en una planta nominal de fácil control.

Esta técnica ha demostrado ser efectiva en aplicaciones de control de movimiento y procesos industriales, donde las perturbaciones y variaciones paramétricas son comunes.

---
## Definiciones

> 🔑 **_ADRC (Active Disturbance Rejection Control)_**: Estrategia de control robusto que estima y cancela activamente perturbaciones mediante un observador de estados extendido (ESO), requiriendo mínimo conocimiento del modelo de la planta. Combina realimentación no lineal y compensación en tiempo real de incertidumbres.

> 🔑 **_Observador de Estados Extendido (ESO)_**: Componente clave del ADRC que expande el espacio de estados para incluir la "perturbación total" como estado adicional, permitiendo su estimación y posterior cancelación. Opera bajo el principio de que cualquier desviación del modelo nominal puede tratarse como perturbación.

> 🔑 **_Perturbación Total (ε(t))_**: Término agregado que engloba dinámicas no modeladas, variaciones paramétricas y perturbaciones externas. El ESO estima ε(t) como un estado extendido, simplificando el diseño del controlador.

## Componentes del ADRC

El **Control por Rechazo Activo de Perturbaciones (ADRC)** se estructura en tres componentes fundamentales que trabajan sinérgicamente para lograr un control robusto independiente del modelo preciso de la planta. El **Generador de Trayectorias** transforma referencias abruptas en perfiles suaves, preservando los actuadores de esfuerzos bruscos. El **Observador de Estados Extendido (ESO)**, corazón del ADRC, estima en tiempo real tanto los estados no medibles como las perturbaciones totales (internas y externas), agrupándolas en una única señal a compensar. Finalmente, la **Ley de Control** combina realimentación de estados y cancelación activa de perturbaciones, simplificando el sistema a una cadena de integradores nominales. Esta arquitectura permite controlar sistemas complejos con sólo conocer su orden dinámico y una aproximación gruesa de su ganancia, demostrando especial eficacia en sistemas no lineales, de parámetros variables o bajo perturbaciones significativas, superando así limitaciones clásicas del control PID tradicional.

### 1. Generador de Trayectorias
Define la referencia deseada para el sistema. Ejemplo:

Para un sistema de segundo orden, el generador puede ser:  

#### 1.1 Formulación Matemática
```math
\begin{aligned}
&\text{Sistema de 2do orden:} \\
&\ddot{y}^* + 2\zeta\omega_n\dot{y}^* + \omega_n^2y^* = \omega_n^2r(t) \\
&\text{Donde:} \\
&\quad \omega_n = \text{Frecuencia natural} \\
&\quad \zeta = \text{Factor de amortiguamiento}
\end{aligned}
```

#### 1.1 Implementación en Python

```math
import numpy as np

def generate_trajectory(t, r, wn, zeta):
    """Genera trayectoria suavizada para referencia escalón"""
    y = np.zeros_like(t)
    yd = np.zeros_like(t)
    ydd = np.zeros_like(t)
    
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        ydd[i] = wn**2*(r[i] - y[i-1]) - 2*zeta*wn*yd[i-1]
        yd[i] = yd[i-1] + ydd[i]*dt
        y[i] = y[i-1] + yd[i]*dt
        
    return y, yd, ydd
```

#### 1.2 Ejemplo de Uso

```math
import matplotlib.pyplot as plt

t = np.linspace(0, 5, 500)
r = np.ones_like(t)   Referencia escalón
y, yd, ydd = generate_trajectory(t, r, wn=2.0, zeta=0.7)

plt.figure(figsize=(10,6))
plt.plot(t, r, 'r--', label='Referencia')
plt.plot(t, y, 'b-', label='Trayectoria generada')
plt.legend(); plt.grid(True)
plt.title('Generador de Trayectorias ADRC')
plt.show()
```

### 2. Observador de Estados Extendido (ESO)

El **Observador de Estados Extendido (ESO** es el componente central del control ADRC que permite estimar y compensar perturbaciones en tiempo real sin requerir un modelo preciso del sistema. Su funcionamiento se basa en expandir el espacio de estados tradicional para incluir no solo las variables de estado físicas, sino también un "estado extendido" que representa la perturbación total que afecta al sistema (incluyendo dinámicas no modeladas, perturbaciones externas y variaciones paramétricas).

El ESO opera mediante un conjunto de ecuaciones diferenciales acopladas que comparan continuamente la salida real del sistema con la salida estimada, ajustando dinámicamente sus estados internos. Para un sistema de segundo orden, por ejemplo, utiliza tres ecuaciones: dos para estimar los estados físicos (posición y velocidad) y una tercera para estimar la perturbación agregada. Las ganancias del observador (β) se sintonizan típicamente en función de un único parámetro (el ancho de banda ωₒ), lo que simplifica notablemente su implementación práctica.

La verdadera potencia del ESO radica en su capacidad para transformar virtualmente cualquier sistema en una cadena simple de integradores, independientemente de sus no linealidades o perturbaciones. Esto se logra estimando la perturbación total en tiempo real y cancelándola directamente en la ley de control. Como resultado, el controlador solo necesita manejar la dinámica nominal simplificada, haciendo que el sistema en lazo cerrado sea inherentemente robusto y fácil de sintonizar.

#### 2.1 Implementación

Estima estados no medibles y perturbaciones. Para un sistema lineal:  

```math
\begin{cases}
\dot{z}_1 = z_2 + \beta_1(y - z_1) \\
\dot{z}_2 = z_3 + \beta_2(y - z_1) + b_0u \\
\dot{z}_3 = \beta_3(y - z_1)
\end{cases}
```

#### 2.2 Demostracion en MATLAB

```math
% MATLAB: Cálculo de ganancias para ancho de banda wo
n = 2;  % Orden del sistema
wo = 30; % Frecuencia del observador
beta = zeros(n+1,1);
for i = 1:n+1
    beta(i) = factorial(n+1)/(factorial(i)*factorial(n+1-i)) * wo^i;
end
```

### 3. Ley de Control del ADRC

El tercer pilar del ADRC es su innovadora **Ley de Control**, que combina de manera elegante la realimentación de estados con la cancelación activa de perturbaciones. A diferencia de los esquemas de control tradicionales que intentan rechazar perturbaciones después de que afectan al sistema, el ADRC las anticipa y neutraliza mediante un enfoque de dos etapas:

1. **Compensación de Perturbaciones**: Utiliza la estimación de la perturbación total (zₙ₊₁) proporcionada por el ESO para cancelarla directamente en la señal de control.
2. **Realimentación de Estados**: Aplica una ley de control convencional (usualmente PD) sobre el sistema nominal libre de perturbaciones.

La forma general de la ley es:

$u = \frac{u_0 - z_{n+1}}{b_0}$

## Ejercicios Resueltos

### Ejercicio 1: Diseño de un ADRC para un sistema masa-resorte-amortiguador

**Planta**:  
\[ M\ddot{y} + B\dot{y} + Ky = u(t) + w(t) \]  
**Paso 1**: Reformular como:  
\[ \ddot{y} = \frac{u}{M} + \epsilon(t), \quad \epsilon(t) = -\frac{B}{M}\dot{y} - \frac{K}{M}y + w(t) \]  
**Paso 2**: Diseñar ESO para estimar \( \epsilon(t) \).  
**Paso 3**: Implementar la ley de control con polos en \( s = -15 \) (críticamente amortiguado).

---

## Ejercicios Adicionales

### Ejercicio 3: Sistema no lineal con acoplamiento trigonométrico

Dado el sistema:  
$$ \dot{x} = -k_1 x - k_2 \sin(x) + c u $$  

#### 1. Linealización alrededor del punto de operación

**Paso 1:** Definir punto de operación  
Sea $(x_0, u_0)$ tal que $\dot{x} = 0$:  
$0 = -k_1 x_0 - k_2 \sin(x_0) + c u_0$

**Paso 2:** Expansión en serie de Taylor  
$\Delta \dot{x} \approx \left.\frac{\partial f}{\partial x}\right|_{x_0} \Delta x + \left.\frac{\partial f}{\partial u}\right|_{u_0} \Delta u$  
Donde:  
$\frac{\partial f}{\partial x} = -k_1 - k_2 \cos(x), \quad \frac{\partial f}{\partial u} = c$

**Resultado linealizado:**  
$\Delta \dot{x} = (-k_1 - k_2 \cos(x_0)) \Delta x + c \Delta u$

#### 2. Diseño de NADRC con función `fal(e, α, δ)`

**Estructura del controlador:**  
```python
def fal(e, alpha, delta):
    return np.abs(e)**alpha * np.sign(e) if np.abs(e) > delta else e/(delta**(1-alpha))
```

# Parámetros NADRC
alpha = 0.5  # Factor no lineal
delta = 0.1  # Zona lineal
b0 = c       # Ganancia aproximada

### Ejercicio 3: Variación paramétrica
Simular un ADRC para:  
\[ \ddot{y} = (4.75 - 4.5y)u + 0.7\dot{y} - 0.25y \]  
Evaluar desempeño con perturbaciones en rampa y sinusoidal.

---

## Conclusiones

- El ADRC ofrece robustez frente a incertidumbres y perturbaciones, superando limitaciones del PID.  
- Su implementación requiere sintonizar menos parámetros que un controlador convencional.  
- Aplicaciones industriales incluyen robótica, control de motores y procesos químicos.  

**Desafíos**:  
- Complejidad computacional en sistemas de alto orden.  
- Requiere ajuste cuidadoso del ESO para evitar ruido en la estimación.  

---

## Referencias

1. Gao, Z. (2014). *Active Disturbance Rejection Control: A Paradigm Shift in Feedback Control Design*. IEEE.  
2. Universidad ECCI. (2025). *Material del curso Control de Movimiento*.  
3. Guo, B. Z., & Zhao, Z. L. (2016). *On Convergence of Nonlinear Active Disturbance Rejection Control*. SIAM Journal on Control and Optimization.  
