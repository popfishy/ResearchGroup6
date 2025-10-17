# core/scenario_parser.py

import xml.etree.ElementTree as ET
from typing import Dict, List, Any, Optional

# 从您的项目中导入 PositionConvert 和数据类
from communication.scripts.WGS84toCartesian import PositionConvert
from .utils import EnemyTarget, TargetStatus
from .uav import UAV

class ScenarioParser:
    """
    解析战场态势XML文件，提取我方无人机和敌方目标信息，
    并使用提供的坐标转换工具转换为局部ENU坐标系。
    """
    def __init__(self, xml_file_path: str):
        self.xml_file_path = xml_file_path
        self.tree = ET.parse(xml_file_path)
        self.root = self.tree.getroot()
        self.converter: Optional[PositionConvert] = None

    def parse(self) -> Dict[str, Any]:
        """
        主解析方法。

        Returns:
            一个字典，包含:
            - 'uavs': UAV对象列表
            - 'enemies': EnemyTarget对象列表
            - 'uav_groups': {group_id: [uav_ids]} 的字典
        """
        uav_data_raw = []
        enemy_data_raw = []

        # 遍历XML中所有的 <entity> 标签
        for entity in self.root.findall('.//entity'):
            has_pos = entity.find('.//hasPosScenario')
            if has_pos is None or has_pos.text != 'true':
                continue # 过滤掉没有实际位置的抽象实体

            entity_info = self._parse_entity_raw(entity)
            if not entity_info:
                continue
            
            camp = entity.get('camp')
            if camp == '1' and '无人机' in entity_info['type_name']:
                uav_data_raw.append(entity_info)
            elif camp == '2':
                enemy_data_raw.append(entity_info)
        
        if not uav_data_raw:
            print("警告: XML中未解析到任何我方无人机实体。")
            return {'uavs': [], 'enemies': [], 'uav_groups': {}}

        # 1. 设定坐标原点（使用解析到的第一架无人机的位置）并初始化转换器
        origin_lla = (uav_data_raw[0]['lon'], uav_data_raw[0]['lat'], uav_data_raw[0]['alt'])
        self.converter = PositionConvert(ref_lon=origin_lla[0], ref_lat=origin_lla[1], ref_alt=origin_lla[2])
        print(f"坐标转换器初始化成功，原点 (LLA): {origin_lla}")

        # 2. 创建UAV对象并分组
        uavs = []
        uav_groups = {}
        for data in uav_data_raw:
            # 使用 WGS84toENU 转换，X=East, Y=North, Z=Up
            e, n, u = self.converter.WGS84toENU(data['lon'], data['lat'], data['alt'])
            uav = UAV(uav_id=data['id'], initial_position=(e, n, u))
            uavs.append(uav)
            
            group_id = data.get('group_id')
            if group_id:
                if group_id not in uav_groups:
                    uav_groups[group_id] = []
                uav_groups[group_id].append(data['id'])

        # 3. 创建EnemyTarget对象
        enemies = []
        for data in enemy_data_raw:
            e, n, u = self.converter.WGS84toENU(data['lon'], data['lat'], data['alt'])
            enemy = EnemyTarget(
                id=data['id'],
                position=(e, n, u),
                target_type=data.get('type_name', 'Unknown'),
                threat_level=self._estimate_threat(data.get('name', '')),
                status=TargetStatus.DETECTED
            )
            enemies.append(enemy)

        return {
            'uavs': uavs,
            'enemies': enemies,
            'uav_groups': uav_groups
        }

    def _parse_entity_raw(self, entity_element: ET.Element) -> Optional[Dict[str, Any]]:
        """从一个 <entity> 标签中提取原始信息"""
        try:
            basis = entity_element.find('basis')
            pos = basis.find('position')
            
            return {
                'id': int(basis.find('id').text),
                'lon': float(pos.find('longitude').text),
                'lat': float(pos.find('latitude').text),
                'alt': float(pos.find('altitude').text),
                'name': basis.find('objectname').text,
                'group_id': int(basis.find('afdiliate-id').text) if basis.find('afdiliate-id') is not None else None,
                'type_name': entity_element.get('entity-name', 'Unknown')
            }
        except (AttributeError, ValueError) as e:
            # 如果缺少必要标签或转换失败，则忽略此实体
            object_name = entity_element.find('.//objectname')
            print(f"解析实体 {object_name.text if object_name is not None else 'Unknown'} 失败: {e}")
            return None

    # TODO:根据仿真修改
    def _estimate_threat(self, name: str) -> int:
        """根据目标名称简单评估威胁等级"""
        name = name.lower()
        if '爱国者' in name or '干扰' in name: return 5
        if '装甲车' in name: return 4
        if '哨所' in name: return 3
        if '士兵' in name: return 2
        return 1
