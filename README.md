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

> 🔑 **_Ley de Control ADRC_**:
> ```math
> u = \frac{u_0 - \hat{ε}(t)}{b_0}
> ```
> donde \( u_0 \) es la señal de control nominal y \( b_0 \) la ganancia aproximada de la planta. Esta estructura transforma el sistema en una cascada de integradores.

> 🔑 **_Función fal(e, α, δ)_**: Función no lineal usada en NADRC para manejar errores grandes/pequeños de forma diferenciada:
> ```python
> def fal(e, alpha, delta):
>     return e/(delta**(1-alpha)) if abs(e)<=delta else abs(e)**alpha*sign(e)
> ```

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




### 2. Observador de Estados Extendido (ESO)
Estima estados no medibles y perturbaciones. Para un sistema lineal:  
\[
\begin{cases} 
\dot{z}_1 = z_2 + L_1(y - z_1) \\ 
\dot{z}_2 = z_3 + b_0 u + L_2(y - z_1) \\ 
\dot{z}_3 = L_3(y - z_1) 
\end{cases}
\]  
Aquí, \( z_3 \) estima la "perturbación total" \( \epsilon(t) \).

### 3. Ley de Control
Combina realimentación de estados y compensación de perturbaciones:  
\[ u = \frac{u_0 - z_3}{b_0}, \quad u_0 = k_1(y^* - z_1) - k_2 z_2 \]

---

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

### Ejercicio 2: Sistema no lineal
Dado el sistema:  
\[ \dot{y} = -a_1 y - a_0 y^3 + b u \]  
1. Linealizar alrededor de un punto de operación.  
2. Diseñar un ADRC no lineal (NADRC) usando la función `fal(e, α, δ)`.  

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
