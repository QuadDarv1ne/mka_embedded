#!/usr/bin/env python3
"""
Telemetry Visualizer
Визуализатор телеметрии для анализа работы спутника

Генерирует:
- Графики временных рядов
- 3D визуализацию ориентации
- Отчёты в HTML формате
"""

import json
import math
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from datetime import datetime
import os


@dataclass
class TelemetryPoint:
    """Точка телеметрии"""
    timestamp: float
    parameters: Dict[str, float] = field(default_factory=dict)


class TelemetryDatabase:
    """
    База данных телеметрии для хранения и анализа.
    """
    
    def __init__(self):
        self.data: List[TelemetryPoint] = []
        self.parameter_info: Dict[str, dict] = {}
    
    def add_point(self, point: TelemetryPoint):
        """Добавление точки телеметрии."""
        self.data.append(point)
        
        # Обновление информации о параметрах
        for name, value in point.parameters.items():
            if name not in self.parameter_info:
                self.parameter_info[name] = {
                    'min': value,
                    'max': value,
                    'count': 1,
                    'sum': value
                }
            else:
                info = self.parameter_info[name]
                info['min'] = min(info['min'], value)
                info['max'] = max(info['max'], value)
                info['count'] += 1
                info['sum'] += value
    
    def get_parameter_stats(self, name: str) -> Optional[dict]:
        """Получение статистики по параметру."""
        if name not in self.parameter_info:
            return None
        
        info = self.parameter_info[name]
        return {
            'min': info['min'],
            'max': info['max'],
            'avg': info['sum'] / info['count'],
            'count': info['count']
        }
    
    def get_time_series(self, name: str) -> tuple:
        """Получение временного ряда для параметра."""
        times = []
        values = []
        
        for point in self.data:
            if name in point.parameters:
                times.append(point.timestamp)
                values.append(point.parameters[name])
        
        return times, values
    
    def load_from_json(self, filename: str):
        """Загрузка из JSON файла."""
        with open(filename, 'r') as f:
            raw_data = json.load(f)
        
        for item in raw_data:
            point = TelemetryPoint(
                timestamp=item.get('timestamp', 0),
                parameters=item.get('parameters', {})
            )
            self.add_point(point)
    
    def save_to_json(self, filename: str):
        """Сохранение в JSON файл."""
        raw_data = [
            {
                'timestamp': p.timestamp,
                'parameters': p.parameters
            }
            for p in self.data
        ]
        
        with open(filename, 'w') as f:
            json.dump(raw_data, f, indent=2)


class TelemetryVisualizer:
    """
    Визуализатор телеметрии.
    
    Генерирует HTML отчёты с графиками без внешних зависимостей.
    Использует Chart.js из CDN.
    """
    
    HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title}</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #1a1a2e;
            color: #eee;
            padding: 20px;
        }}
        .container {{
            max-width: 1400px;
            margin: 0 auto;
        }}
        h1 {{
            color: #4fc3f7;
            margin-bottom: 10px;
        }}
        h2 {{
            color: #81d4fa;
            margin: 20px 0 10px 0;
            border-bottom: 1px solid #333;
            padding-bottom: 5px;
        }}
        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }}
        .stat-card {{
            background: #16213e;
            border-radius: 10px;
            padding: 15px;
            border-left: 4px solid #4fc3f7;
        }}
        .stat-card.warning {{
            border-left-color: #ff9800;
        }}
        .stat-card.error {{
            border-left-color: #f44336;
        }}
        .stat-card h3 {{
            color: #888;
            font-size: 12px;
            text-transform: uppercase;
            margin-bottom: 5px;
        }}
        .stat-card .value {{
            font-size: 24px;
            font-weight: bold;
            color: #fff;
        }}
        .stat-card .unit {{
            color: #888;
            font-size: 14px;
        }}
        .chart-container {{
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            margin: 15px 0;
        }}
        .chart-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 20px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            background: #16213e;
            border-radius: 10px;
            overflow: hidden;
            margin: 15px 0;
        }}
        th, td {{
            padding: 12px 15px;
            text-align: left;
            border-bottom: 1px solid #333;
        }}
        th {{
            background: #1a1a2e;
            color: #4fc3f7;
            font-weight: 600;
        }}
        tr:hover {{
            background: #1a1a2e;
        }}
        .status-ok {{ color: #4caf50; }}
        .status-warning {{ color: #ff9800; }}
        .status-error {{ color: #f44336; }}
        .header-info {{
            color: #888;
            margin-bottom: 20px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>{title}</h1>
        <div class="header-info">
            <p>Период: {start_time} - {end_time}</p>
            <p>Точек телеметрии: {point_count}</p>
        </div>
        
        {content}
    </div>
    
    <script>
        {scripts}
    </script>
</body>
</html>
"""
    
    def __init__(self, database: TelemetryDatabase):
        self.db = database
    
    def generate_report(self, title: str = "Telemetry Report") -> str:
        """
        Генерация HTML отчёта.
        
        Args:
            title: Заголовок отчёта
        
        Returns:
            HTML строка
        """
        content = ""
        scripts = ""
        chart_id = 0
        
        # Карточки статистики
        content += self._generate_stats_section()
        
        # Графики по группам параметров
        param_groups = self._group_parameters()
        
        for group_name, params in param_groups.items():
            content += f"<h2>{group_name}</h2>"
            content += '<div class="chart-grid">'
            
            for param in params[:4]:  # Максимум 4 графика в группе
                chart_id += 1
                chart_html, chart_script = self._generate_chart(param, chart_id)
                content += chart_html
                scripts += chart_script
            
            content += '</div>'
        
        # Таблица параметров
        content += self._generate_params_table()
        
        # Временные метки
        start_time = datetime.fromtimestamp(self.db.data[0].timestamp).strftime('%Y-%m-%d %H:%M:%S') if self.db.data else "N/A"
        end_time = datetime.fromtimestamp(self.db.data[-1].timestamp).strftime('%Y-%m-%d %H:%M:%S') if self.db.data else "N/A"
        
        return self.HTML_TEMPLATE.format(
            title=title,
            start_time=start_time,
            end_time=end_time,
            point_count=len(self.db.data),
            content=content,
            scripts=scripts
        )
    
    def _generate_stats_section(self) -> str:
        """Генерация секции статистики."""
        stats_cards = ""
        
        # Ключевые параметры для карточек
        key_params = [
            ('VBAT', 'В', 'Напряжение батареи'),
            ('BAT_CHARGE', '%', 'Заряд батареи'),
            ('TEMP_INTERNAL', '°C', 'Температура внутри'),
            ('CPU_LOAD', '%', 'Загрузка CPU'),
        ]
        
        for param, unit, label in key_params:
            stats = self.db.get_parameter_stats(param)
            if stats:
                # Определение статуса
                card_class = ""
                if param == 'VBAT':
                    if stats['min'] < 6.0:
                        card_class = "error"
                    elif stats['min'] < 6.5:
                        card_class = "warning"
                elif param == 'BAT_CHARGE':
                    if stats['min'] < 20:
                        card_class = "error"
                    elif stats['min'] < 40:
                        card_class = "warning"
                
                stats_cards += f"""
                <div class="stat-card {card_class}">
                    <h3>{label}</h3>
                    <div class="value">{stats['avg']:.1f} <span class="unit">{unit}</span></div>
                    <small>Min: {stats['min']:.1f} | Max: {stats['max']:.1f}</small>
                </div>
                """
        
        return f'<div class="stats-grid">{stats_cards}</div>'
    
    def _group_parameters(self) -> Dict[str, List[str]]:
        """Группировка параметров по категориям."""
        groups = {
            'Электропитание': [],
            'Температуры': [],
            'Ориентация (ADCS)': [],
            'Связь': [],
            'Система': []
        }
        
        for param in self.db.parameter_info.keys():
            if any(x in param.upper() for x in ['VBAT', 'BAT', 'SOLAR', 'V3V3', 'V5V', 'I3V3', 'I5V']):
                groups['Электропитание'].append(param)
            elif 'TEMP' in param.upper():
                groups['Температуры'].append(param)
            elif any(x in param.upper() for x in ['OMEGA', 'MAG', 'Q', 'WHEEL', 'QUAT']):
                groups['Ориентация (ADCS)'].append(param)
            elif any(x in param.upper() for x in ['RSSI', 'TX', 'RX', 'COMM']):
                groups['Связь'].append(param)
            else:
                groups['Система'].append(param)
        
        # Удаляем пустые группы
        return {k: v for k, v in groups.items() if v}
    
    def _generate_chart(self, param: str, chart_id: int) -> tuple:
        """Генерация графика для параметра."""
        times, values = self.db.get_time_series(param)
        
        if not values:
            return "", ""
        
        # Преобразование времени
        labels = [datetime.fromtimestamp(t).strftime('%H:%M:%S') for t in times]
        
        # Нормализация для определения цвета
        stats = self.db.get_parameter_stats(param)
        
        chart_html = f"""
        <div class="chart-container">
            <canvas id="chart{chart_id}"></canvas>
        </div>
        """
        
        # JavaScript для графика
        chart_script = f"""
        (function() {{
            const ctx = document.getElementById('chart{chart_id}').getContext('2d');
            new Chart(ctx, {{
                type: 'line',
                data: {{
                    labels: {json.dumps(labels)},
                    datasets: [{{
                        label: '{param}',
                        data: {json.dumps(values)},
                        borderColor: '#4fc3f7',
                        backgroundColor: 'rgba(79, 195, 247, 0.1)',
                        borderWidth: 2,
                        fill: true,
                        tension: 0.1,
                        pointRadius: 0,
                        pointHitRadius: 10
                    }}]
                }},
                options: {{
                    responsive: true,
                    maintainAspectRatio: true,
                    plugins: {{
                        legend: {{
                            display: true,
                            labels: {{ color: '#fff' }}
                        }},
                        title: {{
                            display: true,
                            text: '{param}',
                            color: '#fff'
                        }}
                    }},
                    scales: {{
                        x: {{
                            ticks: {{ color: '#888', maxTicksLimit: 10 }},
                            grid: {{ color: '#333' }}
                        }},
                        y: {{
                            ticks: {{ color: '#888' }},
                            grid: {{ color: '#333' }}
                        }}
                    }}
                }}
            }});
        }})();
        """
        
        return chart_html, chart_script
    
    def _generate_params_table(self) -> str:
        """Генерация таблицы параметров."""
        rows = ""
        
        for param, info in sorted(self.db.parameter_info.items()):
            stats = self.db.get_parameter_stats(param)
            
            rows += f"""
            <tr>
                <td>{param}</td>
                <td>{stats['min']:.3f}</td>
                <td>{stats['max']:.3f}</td>
                <td>{stats['avg']:.3f}</td>
                <td>{stats['count']}</td>
            </tr>
            """
        
        return f"""
        <h2>Все параметры</h2>
        <table>
            <thead>
                <tr>
                    <th>Параметр</th>
                    <th>Мин</th>
                    <th>Макс</th>
                    <th>Среднее</th>
                    <th>Кол-во</th>
                </tr>
            </thead>
            <tbody>
                {rows}
            </tbody>
        </table>
        """
    
    def save_report(self, filename: str, title: str = "Telemetry Report"):
        """Сохранение отчёта в файл."""
        html = self.generate_report(title)
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(html)


class QuaternionVisualizer:
    """
    Визуализация кватерниона ориентации в текстовом виде.
    """
    
    @staticmethod
    def quaternion_to_ascii_art(q, width=40, height=20):
        """
        Простая ASCII визуализация ориентации.
        Показывает направление оси Z спутника.
        """
        # Преобразование кватерниона в углы Эйлера
        import math
        
        # Roll, Pitch
        sinr_cosp = 2.0 * (q['w'] * q['x'] + q['y'] * q['z'])
        cosr_cosp = 1.0 - 2.0 * (q['x'] * q['x'] + q['y'] * q['y'])
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (q['w'] * q['y'] - q['z'] * q['x'])
        pitch = math.asin(max(-1, min(1, sinp)))
        
        # Нормализация в координаты экрана
        x = int((roll / math.pi + 1) * width / 2)
        y = int((pitch / (math.pi/2) + 1) * height / 2)
        
        # Создание ASCII арта
        grid = [[' ' for _ in range(width)] for _ in range(height)]
        
        # Сетка
        for i in range(0, width, 8):
            for j in range(height):
                grid[j][i] = '·'
        for i in range(0, height, 4):
            for j in range(width):
                grid[i][j] = '·'
        
        # Центр
        grid[height//2][width//2] = '+'
        
        # Точка направления
        x = max(0, min(width-1, x))
        y = max(0, min(height-1, y))
        grid[y][x] = '●'
        
        return '\n'.join(''.join(row) for row in grid)


def example_usage():
    """Пример использования визуализатора."""
    print("=== Telemetry Visualizer Example ===\n")
    
    # Создание тестовой базы данных
    db = TelemetryDatabase()
    
    # Добавление синтетических данных
    import random
    import time
    
    base_time = time.time() - 3600  # Час назад
    
    for i in range(100):
        t = base_time + i * 36  # Каждые 36 секунд
        
        point = TelemetryPoint(
            timestamp=t,
            parameters={
                'VBAT': 7.2 + random.gauss(0, 0.1) - i * 0.002,
                'BAT_CHARGE': 95 - i * 0.3 + random.gauss(0, 1),
                'TEMP_INTERNAL': 25 + 10 * math.sin(i * 0.1) + random.gauss(0, 2),
                'CPU_LOAD': 20 + 5 * math.sin(i * 0.2) + random.gauss(0, 3),
                'OMEGA_X': random.gauss(0.001, 0.0005),
                'OMEGA_Y': random.gauss(0.001, 0.0005),
                'OMEGA_Z': random.gauss(0.001, 0.0005),
                'Q0': math.cos(i * 0.01) + random.gauss(0, 0.01),
                'Q1': random.gauss(0, 0.05),
                'Q2': random.gauss(0, 0.05),
                'Q3': math.sin(i * 0.01) + random.gauss(0, 0.01),
            }
        )
        db.add_point(point)
    
    # Генерация отчёта
    viz = TelemetryVisualizer(db)
    output_path = '/home/z/my-project/download/telemetry_report.html'
    viz.save_report(output_path, "MKA Telemetry Report")
    
    print(f"Отчёт сохранён: {output_path}")
    
    # Пример ASCII визуализации
    print("\nВизуализация ориентации:")
    q = {'w': 0.9, 'x': 0.1, 'y': 0.2, 'z': 0.3}
    print(QuaternionVisualizer.quaternion_to_ascii_art(q))


if __name__ == '__main__':
    example_usage()
