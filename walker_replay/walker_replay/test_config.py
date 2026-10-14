"""Parser of test_config.txt, the per-test metadata that lives next to the definitive bags.

Format (one test per line, ':' separated, first line is a header):

    [test]:[handle_height]:[age]:[height]:[weight]:[user_id]:[gender]:[notes]
    MF_test01:4:44:175:97:MF:m:movimiento en circulos

  test          bag name (MF_test01, LA_test07, ...), the same as the bag folder
  handle_height index (0-4) of the hole the handles were set to, i.e. the column
                of walker_description/config/params.yaml handle_x/y/z
  age           years
  height        cm
  weight        kg
  user_id       two letters (MF, LA, CA, MT)
  gender        m | f
  notes         free text; may itself contain ':'
"""
from dataclasses import dataclass
from os.path import expanduser, isdir, join

CONFIG_FILE_NAME = 'test_config.txt'
N_COLUMNS = 8

# walker_msgs/UserDesc.gender is what the web GUI enters (see walker_web_gui/
# scripts/console_config.py) and what walker_stability compares against.
GENDERS = {'m': 'Masculino', 'f': 'Femenino'}


class ConfigFileError(RuntimeError):
    pass


@dataclass(frozen=True)
class BagConfig:
    test: str
    handle_height: int
    age: int
    height: int
    weight: int
    user_id: str
    gender: str          # 'm' | 'f', as in the file
    notes: str

    @property
    def gender_desc(self):
        """Gender as written in walker_msgs/UserDesc ('Masculino' | 'Femenino')."""
        return GENDERS[self.gender]


def config_file_path(bag_folder):
    """test_config.txt of a base folder (or the path itself if it is the file)."""
    bag_folder = expanduser(str(bag_folder))
    return join(bag_folder, CONFIG_FILE_NAME) if isdir(bag_folder) else bag_folder


def parse_line(line, source='test_config.txt', lineno=0):
    fields = [t.strip() for t in line.rstrip('\n').split(':', N_COLUMNS - 1)]
    where = f"{source}:{lineno}"
    if len(fields) != N_COLUMNS:
        raise ConfigFileError(f"{where}: {len(fields)} columns, expected {N_COLUMNS}: '{line.strip()}'")
    test, handle_height, age, height, weight, user_id, gender, notes = fields
    try:
        ints = [int(v) for v in (handle_height, age, height, weight)]
    except ValueError as e:
        raise ConfigFileError(f"{where}: non numeric handle_height/age/height/weight: {e}") from None
    if gender not in GENDERS:
        raise ConfigFileError(f"{where}: gender '{gender}' must be one of {sorted(GENDERS)}")
    return BagConfig(test, ints[0], ints[1], ints[2], ints[3], user_id, gender, notes)


def read_test_config(path):
    """Whole file as {test name: BagConfig}. `path` may be the base folder."""
    path = config_file_path(path)
    rows = {}
    try:
        with open(path, encoding='utf-8') as f:
            lines = f.readlines()
    except OSError as e:
        raise ConfigFileError(f"cannot read {path}: {e}") from None
    for lineno, line in enumerate(lines, start=1):
        if not line.strip() or line.lstrip().startswith('[test]'):
            continue
        row = parse_line(line, path, lineno)
        if row.test in rows:
            raise ConfigFileError(f"{path}:{lineno}: duplicated test '{row.test}'")
        rows[row.test] = row
    return rows


def read_bag_config(path, bag_name):
    """The row of one bag; fails listing what is there when it is missing."""
    rows = read_test_config(path)
    if bag_name not in rows:
        raise ConfigFileError(
            f"no row for '{bag_name}' in {config_file_path(path)} "
            f"({len(rows)} tests there, e.g. {', '.join(list(rows)[:4])})")
    return rows[bag_name]
