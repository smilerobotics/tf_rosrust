use tf_rosrust::TfListener;

fn main() {
    rosrust::init("listener");
    let listener = TfListener::new();

    let rate = rosrust::rate(1.0);
    while rosrust::is_ok() {
        let tf = listener.lookup_transform("camera", "base_link", rosrust::Time::new());
        println!("{tf:?}");
        rate.sleep();
    }
}
