# Copyright 2026 KAS-lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Export one representative run's requirement fulfillment for PGFPlots."""

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np  # noqa: I100,I201
import yaml  # noqa: I100,I201

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores


# Recorded SUAVE conventions; no SUAVE or generated ROS packages are needed.
THRUSTER_AVAILABILITY = 'thruster_availability'
SEARCH_FOOTPRINT = 'search_footprint'
REACTION_TIME = 'adaptation_reaction_time'
REQUIREMENTS = {
    THRUSTER_AVAILABILITY: {
        'topic': '/requirements/thruster_availability/fulfillment',
        'label': 'Thruster availability',
    },
    SEARCH_FOOTPRINT: {
        'topic': '/requirements/search_footprint/fulfillment',
        'label': 'Search footprint',
    },
    REACTION_TIME: {
        'topic': '/requirements/adaptation_reaction_time/fulfillment',
        'label': 'Adaptation reaction time',
    },
}
# Retain the upstream reader's per-type invalid-sample counters in manifests.
for reaction_type in ('thruster', 'water_visibility', 'battery'):
    REQUIREMENTS[reaction_type + '_reaction_time'] = {
        'topic': f'/requirements/adaptation_reaction_time/{reaction_type}/fulfillment',
    }
MANAGER_LABELS = {
    'rebetmc': 'TacticalRetreat', 'tactical_retreat': 'TacticalRetreat',
    'rosa_bt': 'ROSA-BT', 'bt': 'BT', 'metacontrol': 'Metacontrol',
    'planta': 'PLANTA', 'random': 'Random', 'none': 'None',
}
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


DEFAULT_PRECISION = 6
REQUIREMENT_KEYS = [THRUSTER_AVAILABILITY, SEARCH_FOOTPRINT, REACTION_TIME]
DEFAULT_REQUIREMENTS = [THRUSTER_AVAILABILITY, SEARCH_FOOTPRINT]
# Meaningful only while searching; cut at the first pipeline detection.
SEARCH_ONLY_REQUIREMENTS = {SEARCH_FOOTPRINT}
THRUSTER_RECOVERY_LABELS = {
    'active': 'Activated thruster recovery',
    'inactive': 'Completed thruster recovery',
}
REQUIREMENT_COLORS = {
    THRUSTER_AVAILABILITY: '0072B2',
    SEARCH_FOOTPRINT: 'E69F00',
    REACTION_TIME: '009E73',
}
SELECTION_METRICS = {
    'search_time_s': 'time searching pipeline (s)',
    'distance_inspected_m': 'distance inspected (m)',
    'mean_reaction_time_s': 'mean reaction time (s)',
}
ALTITUDE_NODE = '/f_generate_search_path_node'
ALTITUDE_PARAMETER = 'spiral_altitude'
MAINTAIN_MOTION_TOPIC = '/f_maintain_motion_node/transition_event'
SEARCH_PATH_TOPIC = '/f_generate_search_path_node/transition_event'
EVENT_TOPICS = {
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
    '/parameter_events': 'rcl_interfaces/msg/ParameterEvent',
    MAINTAIN_MOTION_TOPIC: 'lifecycle_msgs/msg/TransitionEvent',
    SEARCH_PATH_TOPIC: 'lifecycle_msgs/msg/TransitionEvent',
    '/pipeline/detected': 'std_msgs/msg/Bool',
    '/mission_metrics/done': 'std_msgs/msg/Bool',
}
FIGURE = 'representative_run.tex'
FIGURE_NO_WATER_VISIBILITY = 'representative_run_no_water_visibility.tex'
INTERVAL_STYLES = {
    'search': 'fill=blue!8',
    'inspect': 'fill=red!8',
    'thruster_failure': 'fill=black, fill opacity=0.18',
}
INTERVAL_LABELS = {
    'search': 'Searching',
    'inspect': 'Inspecting',
    'thruster_failure': 'Thruster failure',
}


@dataclass
class BatchLayout:
    """Input campaigns and the conventional output root."""

    root: Path
    campaigns: list
    results_root: Path


def batch_layout(path):
    """Accept a batch, campaigns directory, campaign, or state.json."""
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise ValueError(f'not found: {path}')
    campaigns = None
    if path.is_file():
        if path.name != 'state.json':
            raise ValueError(f'Expected state.json: {path}')
        root = path.parent
    elif (path / 'runs.json').is_file():
        root = path.parent.parent if path.parent.name == 'campaigns' else path.parent
        campaigns = [path]
    elif (path / 'campaigns').is_dir() or (path / 'state.json').is_file():
        root = path
    elif path.name == 'campaigns':
        root = path.parent
    else:
        raise ValueError(f'Expected a batch, campaigns directory, or campaign: {path}')
    if campaigns is None:
        manifest = root / 'state.json'
        if manifest.is_file():
            state = json.loads(manifest.read_text(encoding='utf-8'))
            if not isinstance(state, dict) or state.get('schema_version') != 2:
                raise ValueError(f'Unsupported state-file schema: {manifest}')
        folder = root / 'campaigns'
        campaigns = sorted(
            child for child in folder.iterdir()
            if (child / 'runs.json').is_file()
        ) if folder.is_dir() else []
    return BatchLayout(root, campaigns, root / 'campaign_results')


def accepted_entries(campaign, manager):
    """Select completed accepted bags in numeric run order, never by glob."""
    campaign = Path(campaign).resolve()
    path = campaign / 'runs.json'
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('schema_version') != 2:
        raise ValueError(f'Unsupported runs schema: {path}')
    if not isinstance(data.get('runs'), dict):
        raise ValueError(f'Invalid runs file: {path}')

    def run_order(run_id):
        parts = run_id.split('_')
        if len(parts) != 3 or parts[0] != 'run' or not all(
                part.isdigit() for part in parts[1:]):
            raise ValueError(f'Invalid run id: {run_id!r}')
        return int(parts[1]), int(parts[2])

    selected = []
    for run_id in sorted(data['runs'], key=run_order):
        entry = data['runs'][run_id]
        if entry['status'] != 'completed' or entry['bag'] is None:
            continue
        if entry['bag'] != 'accepted':
            raise ValueError(f'Completed run without accepted bag: {run_id}')
        relative = (f'rosbags/{entry["adaptation_manager"]}_{entry["mission_name"]}_'
                    f'{run_id}_{entry["attempt_id"]}')
        bag = (campaign / relative).resolve()
        if not bag.is_relative_to(campaign) or bag == campaign:
            raise ValueError(f'Artifact escapes campaign: {relative}')
        if not (bag / 'metadata.yaml').is_file():
            raise ValueError(f'Missing accepted recording metadata: {bag}')
        if entry['adaptation_manager'] == manager:
            selected.append({
                'run_id': run_id, 'attempt_id': entry['attempt_id'],
                'adaptation_manager': manager,
                'bag_path': str(bag.relative_to(campaign)),
            })
    return selected


def metadata_info(bag_dir):
    """Use recorded metadata exactly, including its duration convention."""
    return yaml.safe_load((Path(bag_dir) / 'metadata.yaml').read_text(
        encoding='utf-8'))['rosbag2_bagfile_information']


def read_messages(bag_dir, topics):
    """Yield selected standard messages in recording-time order, without ROS.

    Use a fixed standard type store rather than registering every bag schema:
    unrelated camera and custom SUAVE messages must not need deserialization.
    """
    with Reader(Path(bag_dir)) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        if not connections:
            return
        for connection in connections:
            if connection.msgtype != topics[connection.topic]:
                raise ValueError(f'Unexpected type for {connection.topic}: '
                                 f'{connection.msgtype}')
        for connection, timestamp, raw in reader.messages(connections=connections):
            yield (connection.topic, timestamp,
                   TYPESTORE.deserialize_cdr(raw, connection.msgtype))


def read_bag(entry, campaign):
    """Read recorded fulfillment values, retaining upstream sample filtering."""
    bag_dir = Path(campaign) / entry['bag_path']
    info = metadata_info(bag_dir)
    start_ns = int(info['starting_time']['nanoseconds_since_epoch'])
    duration_ns = int(info['duration']['nanoseconds'])
    if duration_ns <= 0:
        raise ValueError(f'Invalid duration for {bag_dir}')
    series = {key: {'times_ns': [], 'values': []} for key in REQUIREMENTS}
    invalid = {key: 0 for key in REQUIREMENTS}
    keys = {spec['topic']: key for key, spec in REQUIREMENTS.items()}
    topics = {topic: 'std_msgs/msg/Float32' for topic in keys}
    for topic, timestamp, msg in read_messages(bag_dir, topics):
        key = keys[topic]
        value = float(msg.data)
        if not np.isfinite(value) or value < -1.0e-6 or value > 1.0 + 1.0e-6:
            invalid[key] += 1
            continue
        series[key]['times_ns'].append(int(timestamp) - start_ns)
        series[key]['values'].append(float(np.clip(value, 0.0, 1.0)))
    return {**entry, 'bag_dir': str(bag_dir), 'duration_ns': duration_ns,
            'series': series, 'invalid_samples': invalid}


def get_diagnostic_value(msg, key):
    """Return the last numeric diagnostic value matching key."""
    value = None
    for status in msg.status:
        for item in status.values:
            if item.key == key:
                try:
                    value = float(item.value)
                except ValueError:
                    continue
    return value


def bool_value(value):
    """Interpret a metrics CSV boolean."""
    return str(value).strip().lower() in ('1', 'true', 'yes')


def write_rows(path, header, rows, precision=DEFAULT_PRECISION):
    """Write whitespace-delimited PGFPlots data, matching upstream formatting."""
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [' '.join(header)]
    for row in rows:
        lines.append(' '.join(
            f'{row[key]:.{precision}f}' if isinstance(row[key], float)
            else str(row[key]) for key in header))
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=('Export the requirement fulfillment, adaptations and '
                     'context of one representative SUAVE run as PGFPlots '
                     'data tables. No ROS installation is required.'))
    parser.add_argument(
        'path',
        help='Batch, campaigns directory, state.json, or campaign folder.')
    parser.add_argument(
        '--output-dir',
        help=('Output root; each campaign writes to <output-dir>/<campaign>/'
              '<manager>_<run_id>/ (default: <batch>/campaign_results/'
              '<campaign>/representative_run/<manager>_<run_id>).'))
    parser.add_argument(
        '--manager', default='rebetmc',
        help='Manager identifier whose run is exported.')
    parser.add_argument(
        '--run-id',
        help=('Run to export, e.g. run_0_15 (default: the run closest to '
              'the manager median, see README).'))
    parser.add_argument(
        '--requirements', nargs='+', choices=REQUIREMENT_KEYS,
        default=DEFAULT_REQUIREMENTS,
        help='Requirements to export and plot.')
    parser.add_argument(
        '--xmax', type=float,
        help='Mission time (s) at which the preview x axis ends '
             '(default: mission end).')
    parser.add_argument(
        '--required-thrusters', type=int, default=6,
        help='Operational thrusters below which a failure is shaded.')
    parser.add_argument(
        '--precision', type=int, default=DEFAULT_PRECISION,
        help='Decimal places for floating-point table values.')
    parser.add_argument(
        '--no-water-visibility', dest='water_visibility',
        action='store_false',
        help=('Leave the water-visibility panel out of the figure, written '
              f'as {FIGURE_NO_WATER_VISIBILITY} instead of {FIGURE}.'))
    parser.add_argument(
        '--compile', action='store_true',
        help='Compile the preview .tex with pdflatex when available.')
    return parser.parse_args(argv)


def float_or_nan(value):
    """Return a CSV field as float, or NaN when it is not numeric."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


def load_candidates(campaign, manager):
    """Return the manager's accepted runs joined with their metrics rows."""
    selected = accepted_entries(campaign, manager)
    path = Path(campaign) / f'{manager}_suave.csv'
    rows = {}
    if path.is_file():
        with path.open(newline='', encoding='utf-8') as handle:
            rows = {row['attempt_id']: row for row in csv.DictReader(handle)}
    candidates = []
    for entry in selected:
        row = rows.get(entry['attempt_id'])
        if row is None or not bool_value(row.get('pipeline found')):
            continue
        candidates.append({
            **entry,
            **{key: float_or_nan(row.get(column))
               for key, column in SELECTION_METRICS.items()},
        })
    return candidates


def rank_candidates(candidates):
    """Return candidates sorted by summed absolute z-score of the metrics."""
    scores = np.zeros(len(candidates))
    for key in SELECTION_METRICS:
        values = np.asarray([c[key] for c in candidates], dtype=float)
        finite = np.isfinite(values)
        std = np.std(values[finite]) if np.any(finite) else 0.0
        if std <= 0.0:
            continue
        z = np.abs(values - np.median(values[finite])) / std
        scores += np.where(finite, z, np.inf)
    ranked = [{**c, 'score': float(s)} for c, s in zip(candidates, scores)]
    # sorted() is stable, so ties keep runs.json order.
    return sorted(ranked, key=lambda c: c['score'])


def choose_run(candidates, run_id=None):
    """Return the requested run, or the best-ranked one, and the ranking."""
    if not candidates:
        raise ValueError('No accepted runs that found the pipeline')
    ranked = rank_candidates(candidates)
    if run_id is None:
        return ranked[0], ranked
    for candidate in ranked:
        if candidate['run_id'] == run_id:
            return candidate, ranked
    raise ValueError(f'Run {run_id} is not an accepted successful run')


def read_events(bag_dir, start_ns):
    """Read adaptation and context events of one bag, in bag seconds."""
    events = {
        'water_visibility': [],
        'operational_thrusters': [],
        'altitude': [],
        'maintain_motion': [],
        'search_start': None,
        'pipeline_detected': None,
        'mission_done': None,
    }
    for topic, timestamp, msg in read_messages(bag_dir, EVENT_TOPICS):
        t = (int(timestamp) - start_ns) * 1e-9
        if topic == '/diagnostics':
            for key in ('water_visibility', 'operational_thrusters'):
                value = get_diagnostic_value(msg, key)
                if value is not None:
                    events[key].append((t, value))
        elif topic == '/parameter_events':
            if msg.node != ALTITUDE_NODE:
                continue
            # new_parameters is the startup declaration, not an adaptation.
            for param in msg.changed_parameters:
                if param.name == ALTITUDE_PARAMETER:
                    events['altitude'].append((t, param.value.double_value))
        elif topic == MAINTAIN_MOTION_TOPIC:
            # Only the manager's activate/deactivate, not startup configure.
            if msg.start_state.label in ('activating', 'deactivating'):
                events['maintain_motion'].append((t, msg.goal_state.label))
        elif topic == SEARCH_PATH_TOPIC:
            if (msg.goal_state.label == 'active' and
                    events['search_start'] is None):
                events['search_start'] = t
        elif topic == '/pipeline/detected':
            if msg.data and events['pipeline_detected'] is None:
                events['pipeline_detected'] = t
        elif topic == '/mission_metrics/done':
            if msg.data and events['mission_done'] is None:
                events['mission_done'] = t
    return events


def adaptations(events):
    """Return the time-ordered actuation-time adaptations of one run."""
    result = []
    previous = None
    for t, value in events['altitude']:
        if value != previous:
            result.append({'time': t, 'kind': 'altitude',
                           'detail': f'{value:g}',
                           'label': f'Search altitude set to {value:g} m'})
        previous = value
    for t, state in events['maintain_motion']:
        result.append({'time': t, 'kind': 'maintain_motion',
                       'detail': state,
                       'label': THRUSTER_RECOVERY_LABELS[state]})
    result.sort(key=lambda adaptation: adaptation['time'])
    for index, adaptation in enumerate(result, start=1):
        adaptation['index'] = index
    return result


def below_intervals(samples, threshold, end):
    """Return [start, end) intervals where a sampled value is below a limit."""
    intervals = []
    start = None
    for t, value in samples:
        if value < threshold and start is None:
            start = t
        elif value >= threshold and start is not None:
            intervals.append((start, t))
            start = None
    if start is not None:
        intervals.append((start, end))
    return intervals


def context_intervals(events, end, required_thrusters):
    """Return mission-phase and thruster-failure intervals."""
    intervals = []
    detected = events['pipeline_detected']
    if events['search_start'] is not None:
        search_end = detected if detected is not None else end
        intervals.append({'start': events['search_start'],
                          'end': search_end, 'kind': 'search'})
    if detected is not None:
        intervals.append({'start': detected, 'end': end, 'kind': 'inspect'})
    for start, stop in below_intervals(
            events['operational_thrusters'], required_thrusters, end):
        intervals.append({'start': start, 'end': stop,
                          'kind': 'thruster_failure'})
    return intervals


def water_visibility_thresholds(campaign, run_id):
    """Return the run's (visibility threshold, search altitude) pairs."""
    run_idx = run_id.rsplit('_', 1)[-1]
    path = Path(campaign) / f'mission_config_run{run_idx}.yaml'
    if not path.is_file():
        return []
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    parameters = config.get('/mission_metrics', {}).get('ros__parameters', {})
    # Visibility at or above a threshold expects the paired altitude.
    return [(float(t), float(a)) for t, a in zip(
        parameters.get('water_visibility_threshold', []),
        parameters.get('expected_altitude', []))]


def shift_events(events, origin):
    """Return events with every time expressed relative to origin."""
    shifted = {}
    for key, value in events.items():
        if value is None:
            shifted[key] = None
        elif isinstance(value, list):
            shifted[key] = [(t - origin, v) for t, v in value]
        else:
            shifted[key] = value - origin
    return shifted


def series_rows(times_ns, values, origin=0.0):
    """Return time/value rows of one requirement series in seconds."""
    return [{'time': t * 1e-9 - origin, 'value': float(v)}
            for t, v in zip(times_ns, values)]


def start_rows_at_zero(rows):
    """Drop rows before time zero, holding the last earlier value at zero."""
    earlier = [row for row in rows if row['time'] < 0.0]
    kept = [row for row in rows if row['time'] >= 0.0]
    if earlier and not (kept and kept[0]['time'] == 0.0):
        kept.insert(0, {'time': 0.0, 'value': earlier[-1]['value']})
    return kept


def censor_rows(rows, cutoff):
    """Drop rows at or after cutoff, holding the last value up to it."""
    if cutoff is None:
        return rows
    kept = [row for row in rows if row['time'] < cutoff]
    if kept:
        kept.append({'time': cutoff, 'value': kept[-1]['value']})
    return kept


def write_tables(output_dir, record, events, adaptation_list, intervals,
                 requirements, origin, precision):
    """Write every PGFPlots table and return the relative file names."""
    data_dir = output_dir / 'data'
    detected = events['pipeline_detected']
    files = {}
    for key in requirements:
        series = record['series'][key]
        rows = start_rows_at_zero(
            series_rows(series['times_ns'], series['values'], origin))
        if key in SEARCH_ONLY_REQUIREMENTS:
            rows = censor_rows(rows, detected)
        path = data_dir / f'{key}.dat'
        write_rows(path, ['time', 'value'], rows, precision)
        files[key] = path.relative_to(output_dir).as_posix()
    path = data_dir / 'water_visibility.dat'
    write_rows(
        path, ['time', 'value'],
        censor_rows(start_rows_at_zero(
            [{'time': t, 'value': v}
             for t, v in events['water_visibility']]), detected),
        precision)
    files['water_visibility'] = path.relative_to(output_dir).as_posix()
    path = data_dir / 'adaptations.dat'
    write_rows(path, ['index', 'time', 'kind', 'detail'],
                         adaptation_list, precision)
    files['adaptations'] = path.relative_to(output_dir).as_posix()
    path = data_dir / 'intervals.dat'
    write_rows(path, ['start', 'end', 'kind'], intervals, precision)
    files['intervals'] = path.relative_to(output_dir).as_posix()
    return files


def tex_escape(text):
    """Escape underscores for LaTeX text."""
    return text.replace('_', r'\_')


def preview_tex(files, requirements, adaptation_list, intervals, thresholds,
                xmax, title, water_visibility=True):
    """Return a standalone PGFPlots document previewing the exported run."""
    ymin, ymax = -0.03, 1.03
    intervals = [{**i, 'end': min(i['end'], xmax)}
                 for i in intervals if i['start'] < xmax]
    adaptation_list = [a for a in adaptation_list if a['time'] <= xmax]
    lines = [
        r'\documentclass[border=4pt]{standalone}',
        r'\usepackage{pgfplots}',
        r'\pgfplotsset{compat=1.18}',
        r'\usepgfplotslibrary{groupplots}',
    ]
    for key, color in REQUIREMENT_COLORS.items():
        lines.append(rf'\definecolor{{req{key.replace("_", "")}}}'
                     rf'{{HTML}}{{{color}}}')
    lines += [
        r'\begin{document}',
        r'\begin{tikzpicture}',
        r'\begin{groupplot}[',
        rf'  group style={{group size=1 by {2 if water_visibility else 1}, '
        r'vertical sep=0.4cm,',
        r'    xlabels at=edge bottom, xticklabels at=edge bottom},',
        rf'  width=10.5cm, xmin=0, xmax={xmax:.2f},',
        r'  set layers, clip mode=individual,',
        r'  grid=major, grid style={black!12},',
        r'  xlabel={Mission time (s)},',
        r'  tick label style={font=\footnotesize},',
        r'  label style={font=\small},',
        r']',
        rf'\nextgroupplot[height=5.5cm, ymin={ymin}, ymax={ymax},',
        r'  ylabel={Satisfaction degree}, title={' + title + '},',
        r'  legend style={at={(1.03,1)}, anchor=north west,',
        r'    font=\footnotesize, draw=none, name=runlegend},',
        r'  legend cell align=left]',
    ]
    for interval in intervals:
        lines.append(
            rf'\fill[{INTERVAL_STYLES[interval["kind"]]}, '
            r'on layer=axis background] '
            rf'(axis cs:{interval["start"]:.3f},{ymin}) rectangle '
            rf'(axis cs:{interval["end"]:.3f},{ymax});')
    for key in requirements:
        spec = REQUIREMENTS[key]
        lines.append(
            rf'\addplot[const plot, very thick, '
            rf'color=req{key.replace("_", "")}] '
            rf'table[x=time, y=value] {{{files[key]}}};')
        lines.append(rf'\addlegendentry{{{spec["label"]}}}')
    for kind in dict.fromkeys(i['kind'] for i in intervals):
        lines.append(rf'\addlegendimage{{area legend, draw=none, '
                     rf'{INTERVAL_STYLES[kind]}}}')
        lines.append(rf'\addlegendentry{{{INTERVAL_LABELS[kind]}}}')
    if adaptation_list:
        lines.append(r'\addlegendimage{black, dashed}')
        lines.append(r'\addlegendentry{Adaptation}')
    row, previous = 0, None
    for adaptation in adaptation_list:
        t = adaptation['time']
        # Step close labels down so they do not hide each other.
        close = previous is not None and t - previous < 0.03 * xmax
        row = row + 1 if close else 0
        previous = t
        lines.append(rf'\draw[black, dashed] (axis cs:{t:.3f},{ymin}) -- '
                     rf'(axis cs:{t:.3f},{ymax});')
        lines.append(rf'\node[fill=white, inner sep=1pt, font=\scriptsize, '
                     rf'anchor=north, yshift=-{row * 9}pt] at '
                     rf'(axis cs:{t:.3f},{ymax}) {{{adaptation["index"]}}};')
    if water_visibility:
        lines.append(
            r'\nextgroupplot[height=3.2cm, ymin=0, ymax=4.2, '
            r'ylabel={Water vis. (m)}]')
        for interval in intervals:
            lines.append(
                rf'\fill[{INTERVAL_STYLES[interval["kind"]]}, '
                r'on layer=axis background] '
                rf'(axis cs:{interval["start"]:.3f},0) rectangle '
                rf'(axis cs:{interval["end"]:.3f},4.2);')
        lines.append(rf'\addplot[const plot, thick, black] table[x=time, '
                     rf'y=value] {{{files["water_visibility"]}}};')
        for threshold, altitude in thresholds:
            lines.append(rf'\addplot[black!50, densely dotted, '
                         rf'domain=0:{xmax:.2f}, samples=2] {{{threshold}}};')
            lines.append(rf'\node[font=\scriptsize, text=black!70, '
                         rf'anchor=south east, inner sep=1pt] '
                         rf'at (axis cs:{xmax:.2f},{threshold}) '
                         rf'{{altitude {altitude:g}\,m}};')
        for adaptation in adaptation_list:
            t = adaptation['time']
            lines.append(rf'\draw[black, dashed] (axis cs:{t:.3f},0) -- '
                         rf'(axis cs:{t:.3f},4.2);')
    # The adaptation box hangs below the legend, outside the plots.
    mapping = r'\\'.join(
        rf'{a["index"]}: {tex_escape(a["label"])} ({a["time"]:.1f}\,s)'
        for a in adaptation_list) or 'No adaptations'
    lines += [
        r'\end{groupplot}',
        r'\node[draw=black!40, rounded corners, align=left, '
        r'font=\scriptsize, anchor=north west] at '
        r'([yshift=-6pt]runlegend.south west) {' + mapping + '};',
        r'\end{tikzpicture}',
        r'\end{document}',
    ]
    return '\n'.join(lines) + '\n'


def caption_tex(adaptation_list, xmax):
    """Return a caption sentence listing the plotted adaptations."""
    items = [rf'({a["index"]}) {tex_escape(a["label"]).lower()} at '
             rf'{a["time"]:.1f}\,s'
             for a in adaptation_list if a['time'] <= xmax]
    if not items:
        return 'No adaptations.\n'
    if len(items) > 1:
        items[-1] = 'and ' + items[-1]
    return 'Adaptations: ' + ', '.join(items) + '.\n'


def compile_preview(output_dir, tex_name):
    """Compile the preview with pdflatex; return whether it succeeded."""
    if shutil.which('pdflatex') is None:
        print('pdflatex not found; skipping compilation')
        return False
    result = subprocess.run(
        ['pdflatex', '-interaction=nonstopmode', '-halt-on-error', tex_name],
        cwd=output_dir, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        print(result.stdout[-2000:])
        return False
    return True


def export_campaign(campaign, output_root, args):
    """Export one campaign's representative run."""
    candidates = load_candidates(campaign, args.manager)
    chosen, ranked = choose_run(candidates, args.run_id)
    print(f'Campaign: {campaign}')
    print('Best-ranked runs (score = summed |z| from the median):')
    for candidate in ranked[:5]:
        print(f'  {candidate["run_id"]}: score={candidate["score"]:.3f} ' +
              ' '.join(f'{key}={candidate[key]:.2f}'
                       for key in SELECTION_METRICS))
    print(f'Exporting {chosen["run_id"]}')
    record = read_bag(chosen, campaign)
    info = metadata_info(Path(record['bag_dir']))
    start_ns = int(info['starting_time']['nanoseconds_since_epoch'])
    duration_s = record['duration_ns'] * 1e-9
    events = read_events(record['bag_dir'], start_ns)
    # Mission time starts with the search; bag time is the fallback.
    origin = events['search_start'] or 0.0
    events = shift_events(events, origin)
    end = (events['mission_done'] if events['mission_done'] is not None
           else duration_s - origin)
    xmax = min(args.xmax, end) if args.xmax else end
    adaptation_list = adaptations(events)
    intervals = context_intervals(events, end, args.required_thrusters)
    thresholds = water_visibility_thresholds(campaign, chosen['run_id'])

    output_dir = output_root / f'{args.manager}_{chosen["run_id"]}'
    files = write_tables(output_dir, record, events, adaptation_list,
                         intervals, args.requirements, origin,
                         args.precision)
    tex_name = FIGURE if args.water_visibility else FIGURE_NO_WATER_VISIBILITY
    (output_dir / tex_name).write_text(
        preview_tex(files, args.requirements, adaptation_list, intervals,
                    thresholds, xmax, MANAGER_LABELS.get(args.manager, args.manager),
                    args.water_visibility),
        encoding='utf-8')
    (output_dir / 'caption.tex').write_text(
        caption_tex(adaptation_list, xmax), encoding='utf-8')
    manifest = {
        'campaign': str(campaign),
        'options': vars(args),
        'run': {key: chosen[key] for key in chosen if key != 'score'},
        'selection': {
            'method': ('minimum summed |value - median| / SD over ' +
                       ', '.join(SELECTION_METRICS)
                       if args.run_id is None else 'requested'),
            'ranking': [{'run_id': c['run_id'], 'score': c['score']}
                        for c in ranked],
        },
        'time_origin': 'mission start (first search activation)',
        'mission_start_bag_s': origin,
        'bag_duration_s': duration_s,
        'mission_done_s': events['mission_done'],
        'files': files,
        'adaptations': adaptation_list,
        'intervals': intervals,
        'water_visibility_thresholds': [
            {'threshold': t, 'altitude': a} for t, a in thresholds],
        'invalid_samples': record['invalid_samples'],
    }
    (output_dir / 'manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(f'Wrote tables, preview and manifest to {output_dir}')
    if args.compile and compile_preview(output_dir, tex_name):
        print(f'Wrote {output_dir / Path(tex_name).with_suffix(".pdf")}')


def main(argv=None):
    """Export a representative run for every campaign."""
    args = parse_args(argv)
    if args.precision < 0:
        raise ValueError('--precision must be non-negative')
    layout = batch_layout(args.path)
    if not layout.campaigns:
        raise ValueError(f'No campaigns found in {layout.root}')
    for campaign in layout.campaigns:
        if args.output_dir:
            output_root = Path(args.output_dir).expanduser() / campaign.name
        else:
            output_root = (layout.results_root / campaign.name /
                           'representative_run')
        export_campaign(campaign, output_root, args)


if __name__ == '__main__':
    main()
