"""Console entry point: `armdoc report ...` needs no ROS; everything else runs the live doctor."""
import sys


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "report":
        from armdoc.report import main as report_main
        return report_main(argv[1:])
    from armdoc.doctor import main as doctor_main
    return doctor_main(argv)


if __name__ == "__main__":
    sys.exit(main())
