#!/usr/bin/env python

from functools import partial

import rospy
from semantic_map_extraction.srv import GenerateKB, GenerateKBResponse


def is_forniture(string):
    fornitures = [
        'sofa',
        'television',
        'coffeetable',
        'kitchentable',
        'fridge',
        'bookshelf',
        'bed',
        'nightstand',
        'chair',
        'armchair',
        'lamp',
        'trashbin',
        'kitchenshelf',
    ]

    if string in fornitures:
        return True

    return False


def generate_kb(req, saving_path='./test.txt'):
    out_file = open(saving_path, 'w')
    #print('Saving in ' + saving_path)
    width = req.width
    height = req.height
    areas = dict()
    sem_map = dict()

    w = 0
    h = 0
    for cell in req.matrix:
        if cell == '0':
            areas[str(h) + ',' + str(w)] = 'outside'
        else:
            values = cell.split(';')

            if values[0] == '0':
                areas[str(h) + ',' + str(w)] = 'outside'
            else:
                areas[str(h) + ',' + str(w)] = values[0]

        w = (w + 1) % width
        if (w == 0):
            h += 1

    w = 0
    h = 0
    for cell in req.matrix:
        if cell == '0':
            None
        else:
            values = cell.split(';')
            objects = values[1]
            properties = values[3]

            isdoor = False
            direction = None

            dirs = ['up', 'right', 'down', 'left']

            for adj, d in zip(values[2].split('#'), dirs):
                if adj == 'true':
                    isdoor = True
                    direction = d

            i = 0
            objs = objects.split('#')
            props = properties.split('#')
            fornitures = [obj for obj in objs if is_forniture(obj)]

            if len(fornitures) > 1:
                print('Warning: more than one furniture in the same cell')

            for obj, prop in zip(objs, props):
                if obj == '0':
                    continue

                if obj == '':
                    obj = str(i)
                    i += 1

                if not isdoor and 'door' not in obj:
                    if obj not in fornitures:
                        for f in fornitures:
                            sem_map[obj + 'on' + f] = 'on(' + obj + '_1,' + f + ')'

                    sem_map[obj + 'type'] = 'type(' + obj + '_1, ' + obj + ')'
                    sem_map[obj] = 'in(' + obj + '_1,' + areas[str(h) + ',' + str(w)] + ')'
                    sem_map[obj + 'pos'] = 'position(' + obj + '_1,[' + \
                        str(req.cell_centers_x[w]) + ',' + str(req.cell_centers_y[h]) + '])'

                    for p in prop.split('~'):
                        if 'color' in p:
                            sem_map[obj + 'color'] = 'color(' + obj + '_1,' + p.split(':')[1] + ')'
                else:
                    w1 = w
                    w2 = w
                    h1 = h
                    h2 = h

                    if direction == 'up' or direction == 'down':
                        h1 = max(0, h1 - 2)
                        h2 = min(height - 1, h2 + 2)

                    if direction == 'right' or direction == 'left':
                        w1 = max(0, w1 - 2)
                        w2 = min(width - 1, w2 + 2)

                    sem_map[obj + 'type'] = 'type(' + obj + ', door)'
                    sem_map[obj] = 'connects(' + obj + ',' + areas[str(h1) + ',' + str(w1)] + \
                        ',' + areas[str(h2) + ',' + str(w2)] + ')'

                    if 'open' in prop:
                        sem_map[obj + 'properties'] = 'isOpen(' + obj + ', true)'
                    else:
                        sem_map[obj + 'properties'] = 'isOpen(' + obj + ', false)'

        w = (w + 1) % width
        if (w == 0):
            h += 1

    for pred in sem_map:
        if sem_map[pred] == '':
            continue
        out_file.write(sem_map[pred] + '.\n')

    # os.system('sort test.txt >> SPQReL.txt')

    out_file.close()
    #print('Output generated.')
    return GenerateKBResponse('Done.')


if __name__ == '__main__':
    rospy.init_node('create_kb')
    saving_path = rospy.get_param('~save_path', './test.txt')

    ckb = partial(generate_kb, saving_path=saving_path)
    s = rospy.Service('/create_kb', GenerateKB, ckb)
    print('Ready to receive data.')
    rospy.spin()
