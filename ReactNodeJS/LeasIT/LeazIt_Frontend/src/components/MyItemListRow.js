"use strict";

import React from 'react';
import {Card, TableRow, TableColumn, FontIcon, Button, Icon, Text } from 'react-md';
import { Link } from 'react-router-dom';
import { SimpleLink } from './SimpleLink';
import StarRatingComponent from "react-star-rating-component"

import AuthService from '../services/AuthService';



export class MyItemListRow extends React.Component {

    constructor(props) {
        super(props);
        var sum = 0;
        for( var i = 0; i < props.user_evals.length; i++ ){
            sum += parseInt( props.user_evals[i].rating, 10 ); //don't forget to add the base
        }
        var avg = 0;
        if(props.user_evals.length > 0){
             avg = sum/props.user_evals.length;
        }
        avg = 2* avg; // scale from 0-10
        this.state ={
            avg: avg,
        };
        console.log(avg)
    }

    render() {
        return (
            <div className="mdc-card" style={{margin:"2em"}}>
                <div className="mdc-card__media mdc-card__media--square">
                    <div className="mdc-card__media-content">
                        <SimpleLink className="card_link" to={`/my_item/${this.props.items._id}`}>{this.props.items.name}</SimpleLink>
                    </div>
                    <div className="mdc-card__media-content">
                        <SimpleLink to={`/my_item/${this.props.items._id}`}>
                            <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${this.props.items.photos[0]}`}/>
                        </SimpleLink>
                    </div>
                </div>
                <div className="mdc-card__media mdc-card__media--square custom">
                    <StarRatingComponent
                        name="rate1"
                        starCount={10}
                        value={this.state.avg}
                    />
                </div>
                <div className="mdc-card__actions">
                    <div className="mdc-card__action-buttons custom">
                        <SimpleLink  className="card_link" to={`/my_item_bookings/${this.props.items._id}`}>View item bookings</SimpleLink>
                    </div>
                    <div className="mdc-card__action-buttons custom">
                        {AuthService.isAuthenticated() ?
                            <SimpleLink className = "card_link" to={`/edit/${this.props.items._id}`}><FontIcon>mode_edit</FontIcon></SimpleLink>
                            :
                            <SimpleLink to={'/login'} className="card_link"><FontIcon>mode_edit</FontIcon></SimpleLink>
                        }
                    </div>
                    <div className="mdc-card__action-icons">
                        {AuthService.isAuthenticated() ?
                                <button className="button_list" onClick={() => this.props.onDelete(this.props.items._id)} icon>delete</button>

                                 :
                                <Link to={'/login'}><FontIcon>delete</FontIcon></Link>
                        }
                    </div>
                </div>
            </div>
            // <div className= "col-4">
            //     <Card
            //         title={this.props.items.name}
            //         image={`http://localhost:3000/photos/${this.props.items.photos[0]}`}>
            //         {/*<Text style={{marginBottom: 10}}>*/}
            //             {/*{this.props.items.description}*/}
            //         {/*</Text>*/}
            //     </Card>
            //     {/*<div className="row">*/}
            //        {/*<Link to={`/item/${this.props.items._id}`}>*/}
            //            {/*<img className="img-fluid" height={80} width={80} src = {`http://localhost:3000/photos/${this.props.items.photos[0]}`}/>*/}
            //        {/*</Link>*/}
            //     {/*</div>*/}
            //     {/*<div className="row">*/}
            //         {/*<SimpleLink to={`/item/${this.props.items._id}`}>{this.props.items.name}</SimpleLink>*/}
            //     {/*</div>*/}
            //     {/*<div className="row">*/}
            //         {/*<p>{this.props.items.description}</p>*/}
            //     {/*</div>*/}
            //     {/*<div className="row">*/}
            //         {/*<SimpleLink to={`/my_item_bookings/${this.props.items._id}`}>bookings</SimpleLink>*/}
            //     {/*</div>*/}
            //     {/*<div className="row">
            //     {/*</div>*/}
            //     {/*<div className="row">*/}
            //     {/*</div>*/}
            // </div>
        );
    }
}