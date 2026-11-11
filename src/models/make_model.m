function model = make_model(cfg)
switch cfg.model
    case 'fossen'
        model = model_fossen(cfg.h);
    case 'identified'
        model = model_identified(cfg.h);
    otherwise
        error('Unknown model %s.',cfg.model);
end
end
