$(document).ready(function() {
    $('ul.table-of-contents li').click(function () {
        location.href = $(this).attr('data-goto');
    })

    $('ul.table-of-contents li').mouseenter(function () {
        var preview_to_unhide = $(this).attr('data-preview');
        if (preview_to_unhide != undefined) {
            $(preview_to_unhide).fadeIn({
                duration : 200
            });
        }
    })

    $('ul.table-of-contents li').mouseleave(function () {
        var preview_to_hide = $(this).attr('data-preview');
        if (preview_to_hide != undefined) {
            $(preview_to_hide).fadeOut({
                duration : 200
            });
        }
    })
});