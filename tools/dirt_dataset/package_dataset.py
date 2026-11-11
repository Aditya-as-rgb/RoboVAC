#!/usr/bin/env python3
"""Turn the recorded episodes into a train/val/test dataset for Ultralytics YOLO (and a zip for Colab).

    python3 package_dataset.py ~/vacuum_runs/dataset ~/vacuum_runs/dirt_yolo --zip

Input  : the folder written by `collect_dataset.sh` (one sub-folder per episode, classes.txt).
Output : <out>/images/{train,val,test}, <out>/labels/{train,val,test}, <out>/data.yaml,
         <out>/dataset_report.json (and <out>.zip with --zip).

The split is by EPISODE, not by frame: frames from one run look almost identical to their
neighbours, so a random frame split would put near-copies in train and val and the score would
look far better than it is. With fewer than 5 episodes there is nothing to split by, so each
episode is cut in time order instead (first 70 % train, next 15 % val, last 15 % test) and the
report says so.

--classes single  : one class "dirt" (the most useful first model)
--classes kind    : crumbs / dust / spill / hair
"""

import argparse
import json
import os
import random
import shutil
import sys
import zipfile


def read_classes(root):
    with open(os.path.join(root, 'classes.txt')) as f:
        return [ln.strip() for ln in f if ln.strip()]


def find_episodes(root):
    eps = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if os.path.isdir(os.path.join(d, 'images')) and os.path.isdir(os.path.join(d, 'labels')):
            eps.append(name)
    return eps


def frames_of(root, ep):
    """[(stem, n_boxes)] in time order."""
    out = []
    for fn in sorted(os.listdir(os.path.join(root, ep, 'images'))):
        stem, ext = os.path.splitext(fn)
        if ext.lower() != '.jpg':
            continue
        lab = os.path.join(root, ep, 'labels', stem + '.txt')
        if not os.path.exists(lab):
            continue
        with open(lab) as f:
            out.append((stem, sum(1 for ln in f if ln.strip())))
    return out


def split_episodes(episodes, seed=0, fractions=(0.7, 0.15, 0.15)):
    """Episode -> 'train'|'val'|'test'. Needs >= 5 episodes; every split gets at least one."""
    eps = list(episodes)
    random.Random(seed).shuffle(eps)
    n = len(eps)
    n_val = max(1, round(n * fractions[1]))
    n_test = max(1, round(n * fractions[2]))
    out = {}
    for i, e in enumerate(eps):
        out[e] = 'val' if i < n_val else 'test' if i < n_val + n_test else 'train'
    return out


def block_split(n, fractions=(0.7, 0.15, 0.15)):
    a, b = int(n * fractions[0]), int(n * (fractions[0] + fractions[1]))
    return ['train'] * a + ['val'] * (b - a) + ['test'] * (n - b)


def convert_label(line, mapping):
    parts = line.split()
    return ' '.join([str(mapping[int(parts[0])])] + parts[1:])


def package(src, dst, classes_mode='single', seed=0, max_background=0.2, make_zip=False):
    src, dst = os.path.expanduser(src), os.path.expanduser(dst)
    names = read_classes(src)
    if classes_mode == 'single':
        out_names, mapping = ['dirt'], {i: 0 for i in range(len(names))}
    else:
        out_names, mapping = names, {i: i for i in range(len(names))}
    episodes = find_episodes(src)
    if not episodes:
        raise SystemExit('no episodes found in %s' % src)
    by_episode = len(episodes) >= 5
    ep_split = split_episodes(episodes, seed) if by_episode else None

    if os.path.exists(dst):
        shutil.rmtree(dst)
    for sp in ('train', 'val', 'test'):
        os.makedirs(os.path.join(dst, 'images', sp))
        os.makedirs(os.path.join(dst, 'labels', sp))

    rng = random.Random(seed)
    report = {'classes': out_names, 'split': 'by_episode' if by_episode else 'by_time_block (fewer than 5 episodes)',
              'episodes': {}, 'images': {}, 'boxes': {}, 'background_dropped': 0}
    plan = {'train': [], 'val': [], 'test': []}
    for ep in episodes:
        frames = frames_of(src, ep)
        splits = [ep_split[ep]] * len(frames) if by_episode else block_split(len(frames))
        report['episodes'][ep] = ep_split[ep] if by_episode else 'blocks'
        for (stem, nbox), sp in zip(frames, splits):
            plan[sp].append((ep, stem, nbox))
    for sp, items in plan.items():
        pos = [it for it in items if it[2] > 0]
        bg = [it for it in items if it[2] == 0]
        keep = int(len(pos) * max_background / (1 - max_background)) if max_background < 1 else len(bg)
        rng.shuffle(bg)
        report['background_dropped'] += max(0, len(bg) - keep)
        chosen = pos + bg[:keep]
        report['images'][sp] = len(chosen)
        report['boxes'][sp] = 0
        for ep, stem, nbox in chosen:
            name = '%s_%s' % (ep, stem)
            shutil.copyfile(os.path.join(src, ep, 'images', stem + '.jpg'),
                            os.path.join(dst, 'images', sp, name + '.jpg'))
            with open(os.path.join(src, ep, 'labels', stem + '.txt')) as f:
                lines = [convert_label(ln, mapping) for ln in f if ln.strip()]
            with open(os.path.join(dst, 'labels', sp, name + '.txt'), 'w') as f:
                f.write('\n'.join(lines) + ('\n' if lines else ''))
            report['boxes'][sp] += len(lines)

    with open(os.path.join(dst, 'data.yaml'), 'w') as f:
        f.write('path: .\ntrain: images/train\nval: images/val\ntest: images/test\n')
        f.write('names:\n' + ''.join('  %d: %s\n' % (i, n) for i, n in enumerate(out_names)))
    with open(os.path.join(dst, 'dataset_report.json'), 'w') as f:
        json.dump(report, f, indent=2)
    if make_zip:
        zpath = dst.rstrip('/') + '.zip'
        with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_STORED) as z:     # jpg is already compressed
            for folder, _, files in os.walk(dst):
                for fn in files:
                    full = os.path.join(folder, fn)
                    z.write(full, os.path.relpath(full, dst))
        report['zip'] = zpath
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', help='folder of recorded episodes')
    ap.add_argument('dst', help='dataset folder to write (replaced if it exists)')
    ap.add_argument('--classes', choices=['single', 'kind'], default='single')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--max-background', type=float, default=0.2,
                    help='at most this share of each split may be dirt-free images (default 0.2)')
    ap.add_argument('--zip', action='store_true', help='also write <dst>.zip to upload to Colab')
    a = ap.parse_args(argv)
    r = package(a.src, a.dst, a.classes, a.seed, a.max_background, a.zip)
    print('split      :', r['split'])
    for sp in ('train', 'val', 'test'):
        print('%-5s      : %5d images, %5d boxes' % (sp, r['images'][sp], r['boxes'][sp]))
    print('dirt-free images dropped to keep the mix balanced:', r['background_dropped'])
    if 'zip' in r:
        print('zip        :', r['zip'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
